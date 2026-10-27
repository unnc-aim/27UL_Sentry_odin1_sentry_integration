#!/usr/bin/env python3
"""Review recorded wall geometry, fit bounded candidates, and compare their outputs."""
import argparse
import json
import re
import time
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from window_calibration_core import (DistanceModel, density_keep,
                                     fit_distance_model, gravity_lidar,
                                     inside_rectangle, plane_points, project,
                                     wall_normal)


def dump(path, value):
    path.write_text(json.dumps(value, indent=2,
                    ensure_ascii=False), encoding='utf-8')


def load_record(root, item, config):
    folder = (root / item['folder']).resolve()
    measurement = json.loads((folder / 'measurement.json').read_text())
    if not measurement['data_available']:
        raise ValueError(f'{folder.name}: capture is incomplete')
    if max(measurement.get('position_span_m', [0])) > .01:
        raise ValueError(
            f'{folder.name}: wall calibration requires a stationary capture')
    if any(abs(v) > .01 for v in measurement.get('max_command', [0, 0, 0])):
        raise ValueError(f'{folder.name}: motion commands were recorded')
    camera = yaml.safe_load((folder / 'calib.yaml').read_text())['cam_0']
    with np.load(folder / 'capture.npz', allow_pickle=False) as source:
        keys = sorted((k for k in source.files if re.fullmatch(
            r'raw_\d+', k)), key=lambda k: int(k[4:]))
        frames = [source[k].reshape(-1).copy() for k in keys]
        times, extrinsic, poses = source['raw_times'], source['extrinsics'], source['poses']
    if not frames or len(extrinsic) == 0 or len(times) != len(frames):
        raise ValueError(
            f'{folder.name}: raw/extrinsic correspondence is missing')
    n = len(frames[0])
    if any(len(frame) != n for frame in frames):
        raise ValueError('Sensor layout changed during capture')
    indices = np.abs(times[:, None] - extrinsic[None, :, 0]).argmin(axis=1)
    paired = extrinsic[indices]
    tcl = np.median(extrinsic[:, 17:33], axis=0).reshape(4, 4)
    til = np.median(extrinsic[:, 1:17], axis=0).reshape(4, 4)
    up, up_imu = gravity_lidar(poses, extrinsic)
    image_name = measurement['images'][min(
        1, len(measurement['images']) - 1)]['file']
    image = np.asarray(Image.open(folder / image_name))
    normal = np.array([1., 0., 0.])
    geometry = {'normal_lidar': normal.tolist(
    ), 'source': 'target front face assumed perpendicular'}
    edge = None
    if item.get('edge_hints'):
        if not config.get('wall_top_is_level'):
            raise ValueError(
                'Declare the wall-top geometry before estimating its normal')
        normal, pixels, mask, error = wall_normal(
            image, item['edge_hints'], camera, tcl, up)
        edge = (pixels, mask)
        geometry = {'normal_lidar': normal.tolist(), 'source': 'wall top edge and recorded gravity',
                    'yaw_degrees': float(np.degrees(np.arctan2(normal[1], normal[0]))),
                    'inlier_fraction': float(mask.mean()),
                    'edge_fit_p95_degrees': float(np.percentile(error[mask], 95))}
    reference = float(measurement['reference_distance_cm']) / 100
    face_offset = float(config['observation_face_offset_m'])
    if config['reference_mode'] == 'optical_axis':
        offset = (reference + face_offset) * normal[0]
    elif config['reference_mode'] == 'perpendicular':
        offset = reference + face_offset * normal[0]
    else:
        raise ValueError(
            'reference_mode must be optical_axis or perpendicular')
    geometry.update(reference_distance_m=reference, plane_offset_m=float(offset),
                    observation_face_offset_m=face_offset,
                    reference_offset_uncertainty_m=config['reference_offset_uncertainty_m'])
    floor_offset = -config['imu_height_m'] - float(up_imu @ til[:3, 3])
    return dict(item, folder=folder, measurement=measurement, camera=camera,
                frames=frames, times=times, extrinsic=paired, point_count=n, image=image,
                edge=edge, geometry=geometry, normal=normal, offset=offset,
                floor_normal=up, floor_offset=floor_offset, up_imu=up_imu, cache={})


def frame_data(record, index, config):
    if index in record['cache']:
        return record['cache'][index]
    frame = record['frames'][index]
    xyz = np.column_stack([frame[k] for k in ('x', 'y', 'z')]).astype(float)
    confidence = frame['confidence'].astype(float)
    length = np.linalg.norm(xyz, axis=1)
    direction = np.divide(xyz, length[:, None], out=np.zeros_like(
        xyz), where=length[:, None] > 1e-9)
    tcl = record['extrinsic'][index, 17:33].reshape(4, 4)
    til = record['extrinsic'][index, 1:17].reshape(4, 4)
    expected = plane_points(direction, record['normal'], record['offset'])
    camera_points = expected @ tcl[:3, :3].T + tcl[:3, 3]
    pixels = project(camera_points, record['camera'])
    valid = (np.isfinite(xyz).all(axis=1) & (xyz[:, 0] > 0) &
             (confidence >= config['model']['min_confidence']) & (length <= config['filter']['max_range_m']))
    label = valid & (camera_points[:, 2] > 0) & inside_rectangle(
        pixels, record['roi'])
    floor_label = np.zeros(len(xyz), dtype=bool)
    if record.get('floor_roi'):
        floor = plane_points(
            direction, record['floor_normal'], record['floor_offset'])
        floor_camera = floor @ tcl[:3, :3].T + tcl[:3, 3]
        floor_pixels = project(floor_camera, record['camera'])
        floor_label = valid & (floor_camera[:, 2] > 0) & inside_rectangle(
            floor_pixels, record['floor_roi'])
    keep, local = density_keep(xyz, confidence, til, config['filter'])
    result = dict(xyz=xyz, confidence=confidence, directions=direction, expected=expected,
                  label=label, floor_label=floor_label, pixels=pixels, til=til,
                  original_keep=keep, original_local=local)
    record['cache'][index] = result
    return result


def training_samples(record, config):
    # Confidence selects valid samples; it is not a distance predictor.
    blocks = []
    directions = {}
    shifts, uncertainty = [], []
    for index in range(len(record['frames'])):
        data = frame_data(record, index, config)
        ids = np.flatnonzero(data['label'])
        rows = np.column_stack((ids, data['xyz'][ids, 0], data['expected'][ids, 0],
                                data['confidence'][ids], data['directions'][ids]))
        blocks.append(rows)
        shifts.extend((data['expected'][ids, 0] -
                      record['geometry']['reference_distance_m']).tolist())
        denominator = data['directions'][ids] @ record['normal']
        uncertainty.extend(abs(config['reference_offset_uncertainty_m'] * record['normal'][0] *
                               data['directions'][ids, 0] / denominator).tolist())
    if not blocks or not any(len(b) for b in blocks):
        raise ValueError(f"{record['name']}: no labeled wall returns")
    rows = np.vstack(blocks)
    ordered = rows[np.argsort(rows[:, 0], kind='stable')]
    rays, starts, counts = np.unique(
        ordered[:, 0], return_index=True, return_counts=True)
    required = max(
        10, int(np.ceil(config['minimum_frame_coverage'] * len(record['frames']))))
    samples = {}
    for ray, start, count in zip(rays, starts, counts):
        if count < required:
            continue
        points = ordered[start:start + count]
        samples[int(ray)] = points[:, 1:4]
        unit = np.mean(points[:, 4:7], axis=0)
        directions[int(ray)] = unit / np.linalg.norm(unit)
    record.update(samples=samples, directions=directions)
    record['geometry'].update(stable_labeled_directions=len(samples),
                              plane_reference_change_m_p05_p50_p95=np.percentile(
                                  shifts, [5, 50, 95]).tolist(),
                              reference_uncertainty_p95_m=float(np.percentile(uncertainty, 95)))


def draw_geometry(records, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    walls = [r for r in records if r['edge'] is not None]
    fig, axes = plt.subplots(2, 2, figsize=(16, 13), squeeze=False)
    for ax, record in zip(axes.flat, walls):
        ax.imshow(record['image'])
        pixels, mask = record['edge']
        ax.scatter(pixels[mask, 0], pixels[mask, 1], s=5,
                   color='lime', label='Wall top used')
        ax.scatter(pixels[~mask, 0], pixels[~mask, 1], s=5,
                   color='red', label='Excluded edge samples')
        l, r, t, b = record['roi']
        ax.add_patch(Rectangle((l, t), r - l, b - t, fill=False,
                     edgecolor='yellow', linewidth=2))
        if record.get('floor_roi'):
            l, r, t, b = record['floor_roi']
            ax.add_patch(Rectangle((l, t), r - l, b - t, fill=False,
                         edgecolor='cyan', linewidth=2))
        g = record['geometry']
        ax.set_title(f"{record['name']} | wall yaw {g['yaw_degrees']:.2f} deg")
        ax.set_xlim(0, 1600)
        ax.set_ylim(1296, 0)
    axes.flat[0].legend(loc='lower left', fontsize=8)
    fig.tight_layout()
    fig.savefig(output / 'geometry_review.png', dpi=150)
    plt.close(fig)


def distribution(values):
    return np.percentile(values, [50, 95]).tolist() if len(values) else None


def evaluate(record, model, config):
    raw_error, changed_error, full_error, timings = [], [], [], []
    before_counts, after_counts, floor_before, floor_after = [], [], [], []
    selected_rays, corrected_rays = set(), set()
    labeled = corrected = 0
    for index in range(len(record['frames'])):
        data = frame_data(record, index, config)
        start = time.perf_counter()
        result, changed = model.apply(data['xyz'], data['confidence'])
        timings.append((time.perf_counter() - start) * 1000)
        keep, local = (density_keep(result, data['confidence'], data['til'], config['filter'])
                       if changed.any() else (data['original_keep'], data['original_local']))
        mask = data['label']
        active = changed & mask
        labeled += int(mask.sum())
        corrected += int(active.sum())
        selected_rays.update(np.flatnonzero(mask).tolist())
        corrected_rays.update(np.flatnonzero(active).tolist())
        raw_error.extend(abs(data['xyz'][mask, 0] -
                         data['expected'][mask, 0]).tolist())
        full_error.extend(
            abs(result[mask, 0] - data['expected'][mask, 0]).tolist())
        changed_error.extend(
            abs(result[active, 0] - data['expected'][active, 0]).tolist())
        before_counts.append(int((data['original_keep'] & mask).sum()))
        after_counts.append(int((keep & mask).sum()))
        height_before = data['original_local'] @ record['up_imu'] + \
            config['imu_height_m']
        height_after = local @ record['up_imu'] + config['imu_height_m']
        limits = config['evaluation']
        floor_before.append(int((data['floor_label'] & data['original_keep'] &
                                 (height_before > limits['obstacle_height_min_m']) &
                                 (height_before < limits['obstacle_height_max_m'])).sum()))
        floor_after.append(int((data['floor_label'] & keep &
                                (height_after > limits['obstacle_height_min_m']) &
                                (height_after < limits['obstacle_height_max_m'])).sum()))
    minimum = config['evaluation']['minimum_object_points']
    before_array, after_array = np.asarray(
        before_counts), np.asarray(after_counts)
    observed = before_array > 0
    retention = np.minimum(1., after_array[observed] / before_array[observed])
    return {'role': record['role'], 'frames': len(record['frames']), 'labeled_points': labeled,
            'changed_labeled_points': corrected, 'labeled_directions': len(selected_rays),
            'changed_directions': len(corrected_rays),
            'direction_coverage': len(corrected_rays) / max(1, len(selected_rays)),
            'original_error_m_p50_p95': distribution(raw_error),
            'corrected_only_error_m_p50_p95': distribution(changed_error),
            'complete_output_error_m_p50_p95': distribution(full_error),
            'retained_target_points_before_p0_p50_p100': np.percentile(before_counts, [0, 50, 100]).tolist(),
            'retained_target_points_after_p0_p50_p100': np.percentile(after_counts, [0, 50, 100]).tolist(),
            'frames_with_target_before': int(np.count_nonzero(np.array(before_counts) >= minimum)),
            'frames_with_target_after': int(np.count_nonzero(np.array(after_counts) >= minimum)),
            'minimum_target_point_retention': float(retention.min()) if len(retention) else None,
            'floor_false_points_before_p50_p95': distribution(floor_before),
            'floor_false_points_after_p50_p95': distribution(floor_after),
            'application_time_ms_p50_p95': distribution(timings)}


def evaluation_checks(results, limits):
    review = [v for v in results.values() if v['role'] == 'review']
    objects = [v for v in results.values() if v['role'] == 'object']
    return {
        'distance': bool(review) and all(
            v['corrected_only_error_m_p50_p95'] is not None and
            v['corrected_only_error_m_p50_p95'][1] <= limits['distance_error_p95_m']
            for v in review),
        'coverage': bool(review) and all(
            v['direction_coverage'] >= limits['minimum_direction_coverage'] for v in review),
        'target_retention': bool(objects) and all(
            v['frames_with_target_after'] >= v['frames_with_target_before'] and
            (v['minimum_target_point_retention'] is None or
             v['minimum_target_point_retention'] >= limits['minimum_point_retention_fraction'])
            for v in results.values()),
        'object_present_after': bool(objects) and all(
            v['frames_with_target_after'] /
            v['frames'] >= limits['minimum_object_frame_fraction']
            for v in objects),
        'object_correction_observed': bool(objects) and all(
            v['direction_coverage'] >= limits['minimum_direction_coverage'] for v in objects),
        'floor_improvement': any(
            v['floor_false_points_after_p50_p95'][0] < v['floor_false_points_before_p50_p95'][0]
            for v in results.values()),
        'floor_non_regression': all(
            all(a <= b for a, b in zip(v['floor_false_points_after_p50_p95'],
                                       v['floor_false_points_before_p50_p95']))
            for v in results.values()),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--data-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--review-only', action='store_true')
    parser.add_argument(
        '--model', type=Path, help='Evaluate a saved model without fitting new coefficients')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Choose a new output directory')
    config = yaml.safe_load(args.manifest.read_text())
    args.output.mkdir(parents=True)
    (args.output / 'manifest.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    records = [load_record(args.data_root, item, config)
               for item in config['captures']]
    for record in records:
        training_samples(record, config)
        print(
            f"已复核 {record['name']}：{len(record['samples'])} 个持续出现的方向", flush=True)
    dump(args.output / 'geometry.json',
         {r['name']: r['geometry'] for r in records})
    draw_geometry(records, args.output)
    if args.review_only:
        return
    if not config.get('image_reviewed'):
        raise ValueError(
            'Review geometry_review.png and set image_reviewed before fitting')
    training = [r for r in records if r['role'] == 'train']
    if len(training) != 3 or len({r['name'] for r in records}) != len(records):
        raise ValueError(
            'Use three distinct training captures and unique record names')
    models = {}
    methods = [DistanceModel.load(
        args.model).metadata['method']] if args.model else config['methods']
    for method in methods:
        model = DistanceModel.load(args.model) if args.model else fit_distance_model(
            training, method, config['model'])
        if args.model:
            for record in records:
                if record['role'] != 'train' and (record['name'] in model.metadata['training_captures'] or
                                                  float(record['times'][0]) in model.metadata.get('training_start_stamps', [])):
                    raise ValueError(
                        'A fitting record was assigned to a verification role')
        else:
            model.metadata.update(reference_mode=config['reference_mode'],
                                  geometry_assumptions=['level wall top', 'recorded IMU gravity',
                                                        'observation face offset as configured'],
                                  reference_offset_uncertainty_m=config['reference_offset_uncertainty_m'])
        model.save(args.output / f'{method}_preview.npz')
        results = {}
        for record in records:
            results[record['name']] = evaluate(record, model, config)
            print(f"已回放 {method} / {record['name']}", flush=True)
        conditions = evaluation_checks(results, config['evaluation'])
        models[method] = {'model_directions': len(model.rays), 'results': results,
                          'checks': conditions, 'eligible_for_new_field_check': all(conditions.values()),
                          'preview_only': True, 'field_verified': False}
    report = {'models': models, 'model_selection_uses_existing_review_record': True,
              'model_refitted': args.model is None,
              'new_independent_field_record_required': True,
              'metric_scope': 'Static labeled geometry and existing density-filter replay; floor height uses configured IMU height. Nav2 costmaps and native SLAM are not evaluated here.',
              'navigation_configuration_changed': False}
    dump(args.output / 'evaluation.json', report)
    lines = ['# 亚克力窗口离线复核', '',
             '本报告使用既有墙面与实物录制。候选只供独立预览；新的实物验证单独进行。', '',
             '墙面距离参考按图像中水平墙顶、设备重力方向和配置中的尺量约定计算。光学原点偏移保留为测量不确定项。', '',
             '| 方法 | 校正方向数 | 新现场检查条件 |', '| --- | --- | --- |']
    for method, data in models.items():
        lines.append(
            f"| {method} | {data['model_directions']} | {data['eligible_for_new_field_check']} |")
    lines += ['', '详细距离误差、覆盖率、实物保留及地面假点统计见 evaluation.json。',
              '地面指标使用配置中的安装高度，只代表离线几何与密度过滤结果。现场代价地图和原生定位仍需分别核对。']
    (args.output / 'report.md').write_text('\n'.join(lines) + '\n')
    print(f'结果已保存：{args.output}', flush=True)


if __name__ == '__main__':
    main()
