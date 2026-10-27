"""Geometry and bounded distance correction for an installed optical window."""
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def cloud_view(message):
    """Read PointCloud2 fields while honoring row padding and byte order."""
    expected = {'x': 7, 'y': 7, 'z': 7, 'confidence': 4}
    formats = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2',
               5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    fields = {field.name: field for field in message.fields}
    if len(fields) != len(
            message.fields) or message.width == 0 or message.height == 0:
        raise ValueError('Point fields and cloud dimensions must be valid')
    if message.row_step < message.width * \
            message.point_step or len(message.data) != message.row_step * message.height:
        raise ValueError('PointCloud2 row/data layout is inconsistent')
    for name, datatype in expected.items():
        field = fields.get(name)
        if field is None or field.datatype != datatype or field.count != 1:
            raise ValueError(f'Point field {name} has an unsupported layout')
    offsets, types, intervals = [], [], []
    endian = '>' if message.is_bigendian else '<'
    for field in message.fields:
        if field.datatype not in formats or field.count < 1:
            raise ValueError('Point field type or count is invalid')
        scalar = np.dtype(endian + formats[field.datatype])
        dtype = scalar if field.count == 1 else np.dtype(
            (scalar, (field.count,)))
        if field.offset < 0 or field.offset + dtype.itemsize > message.point_step:
            raise ValueError('Point field extends beyond its record')
        offsets.append(field.offset)
        types.append(dtype)
        intervals.append((field.offset, field.offset + dtype.itemsize))
    ordered = sorted(intervals)
    if any(a[1] > b[0] for a, b in zip(ordered, ordered[1:])):
        raise ValueError('Point fields overlap')
    dtype = np.dtype({'names': list(fields), 'formats': types,
                      'offsets': offsets, 'itemsize': message.point_step})
    return np.ndarray((message.height, message.width), dtype=dtype,
                      buffer=message.data, strides=(message.row_step, message.point_step))


def project(points, camera):
    radial = np.linalg.norm(points[:, :2], axis=1)
    theta = np.arctan2(radial, points[:, 2])
    distorted = theta.copy()
    for power in range(2, 8):
        distorted += camera[f'k{power}'] * theta ** power
    scale = np.divide(distorted, radial, out=np.ones_like(
        radial), where=radial > 1e-12)
    xy = points[:, :2] * scale[:, None]
    return np.column_stack((camera['A11'] * xy[:, 0] + camera['A12'] * xy[:, 1] + camera['u0'],
                            camera['A22'] * xy[:, 1] + camera['v0']))


def unproject(pixels, camera):
    y = (pixels[:, 1] - camera['v0']) / camera['A22']
    x = (pixels[:, 0] - camera['u0'] - camera['A12'] * y) / camera['A11']
    radial = np.hypot(x, y)
    theta = radial.copy()
    for _ in range(20):
        value, derivative = theta.copy(), np.ones_like(theta)
        for power in range(2, 8):
            value += camera[f'k{power}'] * theta ** power
            derivative += power * camera[f'k{power}'] * theta ** (power - 1)
        if np.any(derivative <= 0):
            raise ValueError(
                'Camera polynomial is not invertible in the selected image region')
        theta -= (value - radial) / derivative
    scale = np.divide(np.sin(theta), radial, out=np.ones_like(
        radial), where=radial > 1e-12)
    return np.column_stack((x * scale, y * scale, np.cos(theta)))


def gravity_lidar(poses, extrinsic):
    q = np.median(poses[:, 4:8], axis=0)
    if not np.isfinite(q).all() or np.linalg.norm(q) < .5:
        raise ValueError('Valid IMU orientation is required')
    rotation = Rotation.from_quat(q / np.linalg.norm(q)).as_matrix()
    til = np.median(extrinsic[:, 1:17], axis=0).reshape(4, 4)
    up_imu = rotation.T @ np.array([0., 0., 1.])
    up = til[:3, :3].T @ up_imu
    return up / np.linalg.norm(up), up_imu


def wall_normal(image, hints, camera, tcl, gravity, search_pixels=35):
    """Fit a level wall edge; hints identify the visible wall-top neighborhood."""
    hints = np.asarray(hints, dtype=float)
    pixels = []
    for u in range(int(hints[0, 0]), int(hints[-1, 0]) + 1, 5):
        expected = np.interp(u, hints[:, 0], hints[:, 1])
        # Wall/ceiling transition is inspected in the generated review figure.
        column = np.median(image[:, max(0, u - 3):u + 4, 0], axis=1)
        column = gaussian_filter1d(column.astype(float), 2)
        contrast = column[8:] - column[:-8]
        low = max(0, int(expected - search_pixels) - 4)
        high = min(len(contrast), int(expected + search_pixels) - 4)
        if high > low:
            v = low + int(np.argmax(contrast[low:high])) + 4
            if contrast[v - 4] > 12:
                pixels.append((u, v))
    pixels = np.asarray(pixels, dtype=float)
    if len(pixels) < 30:
        raise ValueError(
            'Wall edge contrast or image coverage is insufficient')
    rays = unproject(pixels, camera)
    rng = np.random.default_rng(23)
    best = np.zeros(len(rays), dtype=bool)
    limit = np.sin(np.radians(.2))
    for _ in range(300):
        a, b = rng.choice(len(rays), 2, replace=False)
        if abs(pixels[a, 0] - pixels[b, 0]) < 300:
            continue
        normal = np.cross(rays[a], rays[b])
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        mask = abs(rays @ (normal / length)) < limit
        if mask.sum() > best.sum():
            best = mask
    if best.mean() < .6:
        raise ValueError('Wall-top trace needs a reviewed annotation')
    _, _, vh = np.linalg.svd(rays[best], full_matrices=False)
    edge_normal = vh[-1]
    up = tcl[:3, :3] @ gravity
    up /= np.linalg.norm(up)
    normal = edge_normal - np.dot(edge_normal, up) * up
    normal = tcl[:3, :3].T @ (normal / np.linalg.norm(normal))
    if normal[0] < 0:
        normal = -normal
    error = np.degrees(np.arcsin(np.clip(abs(rays @ edge_normal), 0, 1)))
    return normal, pixels, best, error


def plane_points(directions, normal, offset):
    denominator = directions @ normal
    distance = np.divide(offset, denominator, out=np.full(len(directions), np.nan),
                         where=abs(denominator) > 1e-9)
    distance[distance <= 0] = np.nan
    return directions * distance[:, None]


def inside_rectangle(pixels, rectangle):
    left, right, top, bottom = rectangle
    return ((pixels[:, 0] > left) & (pixels[:, 0] < right) &
            (pixels[:, 1] > top) & (pixels[:, 1] < bottom))


class DistanceModel:
    """Per-direction interpolation confined to measured distance intervals."""

    def __init__(self, metadata, rays, directions, raw_knots, corrected_knots):
        self.metadata = metadata
        if (not isinstance(metadata.get('point_count'), int) or
                not 0 < metadata['point_count'] <= 1000000 or
                not 0 < metadata.get('direction_tolerance_deg', 0) <= 5 or
                not 1 <= metadata.get('min_confidence', 0) <= 65535 or
                not np.isfinite([metadata.get('min_gain', np.nan), metadata.get('max_gain', np.nan)]).all() or
                not 0 < metadata['min_gain'] <= metadata['max_gain'] or
                not isinstance(metadata.get('frame_id'), str) or not metadata['frame_id']):
            raise ValueError('Model limits or sensor layout are invalid')
        self.rays = np.asarray(rays, dtype=np.int64)
        self.directions = np.asarray(directions, dtype=float)
        self.raw_knots = np.asarray(raw_knots, dtype=float)
        self.corrected_knots = np.asarray(corrected_knots, dtype=float)
        n = len(self.rays)
        if (self.directions.shape != (n, 3) or self.raw_knots.shape != (n, 3) or
                self.corrected_knots.shape != (n, 3) or n == 0):
            raise ValueError('Model arrays have inconsistent dimensions')
        if (np.any(np.diff(self.rays) <= 0) or self.rays[0] < 0 or
                self.rays[-1] >= metadata['point_count']):
            raise ValueError(
                'Model directions must be unique, ordered, and in range')
        if not all(np.isfinite(a).all() for a in (
                self.directions, self.raw_knots, self.corrected_knots)):
            raise ValueError('Model values must be finite')
        if (np.any(np.diff(self.raw_knots, axis=1) <= 0) or
                np.any(np.diff(self.corrected_knots, axis=1) <= 0)):
            raise ValueError('Distance knots must increase')
        if not np.allclose(np.linalg.norm(
                self.directions, axis=1), 1, atol=1e-5):
            raise ValueError('Model directions must be unit vectors')
        slopes = np.diff(self.corrected_knots, axis=1) / \
            np.diff(self.raw_knots, axis=1)
        if np.any(slopes < metadata['min_gain']) or np.any(
                slopes > metadata['max_gain']):
            raise ValueError('Model gain is outside its declared interval')

    def save(self, path):
        np.savez_compressed(path, metadata=np.array(json.dumps(self.metadata)), rays=self.rays,
                            directions=self.directions, raw_knots=self.raw_knots,
                            corrected_knots=self.corrected_knots)

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            return cls(json.loads(str(data['metadata'])), data['rays'], data['directions'],
                       data['raw_knots'], data['corrected_knots'])

    def apply(self, xyz, confidence):
        xyz = np.asarray(xyz)
        if xyz.shape != (self.metadata['point_count'], 3) or len(
                confidence) != len(xyz):
            raise ValueError(
                'Point count differs from the recorded sensor layout')
        output = xyz.copy()
        changed = np.zeros(len(xyz), dtype=bool)
        points = xyz[self.rays]
        lengths = np.linalg.norm(points, axis=1)
        direction = np.divide(points, lengths[:, None], out=np.zeros_like(points),
                              where=lengths[:, None] > 1e-9)
        x = points[:, 0]
        valid = (np.isfinite(points).all(axis=1) & (x > 0) &
                 (confidence[self.rays] >= self.metadata['min_confidence']) &
                 (np.einsum('ij,ij->i', direction, self.directions) >=
                  np.cos(np.radians(self.metadata['direction_tolerance_deg']))) &
                 (x >= self.raw_knots[:, 0]) & (x <= self.raw_knots[:, -1]))
        indices = np.flatnonzero(valid)
        interval = (x[indices] > self.raw_knots[indices, 1]).astype(int)
        lo = self.raw_knots[indices, interval]
        hi = self.raw_knots[indices, interval + 1]
        fraction = (x[indices] - lo) / (hi - lo)
        corrected = (self.corrected_knots[indices, interval] * (1 - fraction) +
                     self.corrected_knots[indices, interval + 1] * fraction)
        scaled = points[indices] * (corrected / x[indices])[:, None]
        finite = np.isfinite(scaled).all(axis=1)
        indices = indices[finite]
        output[self.rays[indices]] = scaled[finite]
        changed[self.rays[indices]] = True
        return output, changed


def fit_distance_model(records, method, options):
    n = records[0]['point_count']
    common = sorted(set.intersection(*(set(r['samples']) for r in records)))
    rays, directions, raw_knots, knots = [], [], [], []
    for ray in common:
        blocks = [r['samples'][ray] for r in records]
        raw = np.array([np.median(b[:, 0]) for b in blocks])
        truth = np.array([np.median(b[:, 1]) for b in blocks])
        order = np.argsort(truth)
        raw, truth = raw[order], truth[order]
        if np.any(np.diff(raw) < options['min_raw_gap_m']):
            continue
        if method == 'offset':
            fitted = raw + np.median(truth - raw)
        elif method == 'scale':
            fitted = raw * float(raw @ truth / (raw @ raw))
        elif method == 'piecewise':
            fitted = truth.copy()
        else:
            raise ValueError('Choose offset, scale or piecewise')
        slopes = np.diff(fitted) / np.diff(raw)
        if (np.any(slopes < options['min_gain']) or np.any(slopes > options['max_gain']) or
                np.any(fitted <= 0)):
            continue
        unit = np.mean([r['directions'][ray] for r in records], axis=0)
        unit /= np.linalg.norm(unit)
        rays.append(ray)
        directions.append(unit)
        raw_knots.append(raw)
        knots.append(fitted)
    metadata = dict(options, point_count=n, method=method,
                    training_captures=[r['name'] for r in records], preview_only=True,
                    field_verified=False, confidence_used_for_distance=False, frame_id='lidar')
    metadata['training_start_stamps'] = [
        float(r['times'][0]) for r in records if 'times' in r]
    return DistanceModel(metadata, rays, directions, raw_knots, knots)


def density_keep(xyz, confidence, til, options):
    local = xyz @ til[:3, :3].T + til[:3, 3]
    ranges = np.linalg.norm(xyz, axis=1)
    radius2 = np.einsum('ij,ij->i', local, local)
    valid = (np.isfinite(local).all(axis=1) & (confidence >= options['min_confidence']) &
             (ranges > 0) & (ranges <= options['max_range_m']) &
             (radius2 >= options['self_radius_m'] ** 2))
    indices = np.flatnonzero(valid)
    if len(indices):
        radius = np.nextafter(float(options['neighbor_radius_m']), 0.)
        support = cKDTree(local[indices]).query_ball_point(
            local[indices], radius, return_length=True) - 1
        required = np.ceil(
            options['reference_neighbors'] / np.maximum(radius2[indices], 1e-12))
        near = radius2[indices] < options['filter_range_m'] ** 2
        valid[indices[near & (support < required)]] = False
    return valid, local
