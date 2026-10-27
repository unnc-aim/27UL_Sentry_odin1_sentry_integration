#!/usr/bin/env python3
"""Geometry, bounded correction, message preservation, and preview integration checks."""
import copy
import json
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from preview_window_calibration import Preview, cloud_view, corrected_message  # noqa: E402
from window_calibration import evaluation_checks  # noqa: E402
from window_calibration_core import (  # noqa: E402
    DistanceModel, fit_distance_model, plane_points, project, unproject,
    wall_normal,
)


def candidate():
    options = dict(min_confidence=35, min_raw_gap_m=.01, min_gain=.2, max_gain=6.,
                   direction_tolerance_deg=.15)
    records = []
    for index, (raw, truth) in enumerate(zip([.2, .4, .6], [.5, .8, 1.2])):
        block = np.array(
            [[raw - .001, truth, 100], [raw, truth, 100], [raw + .001, truth, 100]])
        records.append({'name': f'train_{index}', 'point_count': 6,
                        'samples': {1: block, 4: block.copy()},
                        'directions': {1: np.array([1., 0., 0.]), 4: np.array([1., 0., 0.])}})
    return fit_distance_model(records, 'piecewise', options)


def message(big_endian=False):
    msg = PointCloud2()
    msg.header.frame_id = 'lidar'
    msg.header.stamp.sec = 123
    msg.header.stamp.nanosec = 456
    msg.height = 2
    msg.width = 3
    msg.point_step = 24
    msg.row_step = 79
    msg.is_bigendian = big_endian
    msg.fields = [PointField(name=n, offset=o, datatype=t, count=1) for n, o, t in [
        ('x', 0, PointField.FLOAT32), ('y', 4,
                                       PointField.FLOAT32), ('z', 8, PointField.FLOAT32),
        ('confidence', 12, PointField.UINT16), ('intensity', 14, PointField.UINT8),
        ('offset_time', 16, PointField.FLOAT32)]]
    msg.data = bytes([71]) * (msg.row_step * msg.height)
    view = cloud_view(msg)
    view['x'][:] = [[.1, .4, .7], [.3, .5, .3]]
    view['y'][:] = 0
    view['z'][:] = 0
    view['confidence'][:] = 100
    return msg


def check_math():
    camera = dict(A11=730., A12=-.035, A22=730., u0=774., v0=623.,
                  k2=.0016, k3=-.0238, k4=.0137, k5=-.0171, k6=.0077, k7=-.00225)
    points = np.array([[0., 0., 1.], [.5, .2, 1.], [-.8, -.4, 1.]])
    unit = points / np.linalg.norm(points, axis=1)[:, None]
    assert np.allclose(
        unproject(project(points, camera), camera), unit, atol=1e-9)
    normal = np.array([np.cos(.2), np.sin(.2), 0.])
    intersections = plane_points(unit, normal, .8)
    good = np.isfinite(intersections).all(axis=1)
    assert np.allclose(intersections[good] @ normal, .8)

    yaw = np.radians(4)
    lidar_normal = np.array([np.cos(yaw), np.sin(yaw), 0.])
    y = np.linspace(-1.1, 1.1, 4000)
    x = (1. - lidar_normal[1] * y) / lidar_normal[0]
    top = np.column_stack([-y, np.full(len(y), -.5), x])
    uv = project(top, camera)
    uv = uv[np.argsort(uv[:, 0])]
    boundary = np.interp(np.arange(1600), uv[:, 0], uv[:, 1])
    image = np.full((1296, 1600, 3), 20, dtype=np.uint8)
    for u in range(1600):
        image[max(0, int(boundary[u])):, u] = 180
    hints = np.column_stack([np.linspace(350, 1200, 5), np.interp(
        np.linspace(350, 1200, 5), uv[:, 0], uv[:, 1])])
    tcl = np.eye(4)
    tcl[:3, :3] = [[0, -1, 0], [0, 0, -1], [1, 0, 0]]
    found, _, mask, _ = wall_normal(
        image, hints, camera, tcl, np.array([0., 0., 1.]))
    assert mask.mean() > .9
    assert np.degrees(np.arccos(np.clip(found @ lidar_normal, -1, 1))) < .2

    model = candidate()
    xyz = np.array([[.3, 0, 0], [.4, 0, 0], [.7, 0, 0],
                   [.1, 0, 0], [.5, 0, 0], [.2, 0, 0]])
    result, changed = model.apply(xyz, np.full(6, 100))
    assert np.flatnonzero(changed).tolist() == [1, 4]
    assert np.allclose(result[[1, 4], 0], [.8, 1.0])
    assert np.array_equal(result[~changed], xyz[~changed])
    xyz[1, 0] = .1
    xyz[4, 0] = .7
    result, changed = model.apply(xyz, np.full(6, 100))
    assert not changed.any() and np.array_equal(result, xyz)
    xyz[1] = [.4, .1, 0]
    xyz[4] = [.5, 0, 0]
    confidence = np.full(6, 100)
    confidence[4] = 34
    assert not model.apply(xyz, confidence)[1].any()
    with tempfile.TemporaryDirectory() as temp:
        path = Path(temp) / 'model.npz'
        model.save(path)
        loaded = DistanceModel.load(path)
        assert loaded.metadata['confidence_used_for_distance'] is False
        assert np.array_equal(loaded.raw_knots, model.raw_knots)


def check_messages():
    model = candidate()
    for endian in [False, True]:
        source = message(endian)
        original = bytes(source.data)
        output, only, changed = corrected_message(source, model)
        assert bytes(source.data) == original
        assert output.header == source.header and output.row_step == source.row_step
        assert output.fields == source.fields and output.is_bigendian == source.is_bigendian
        assert np.allclose(cloud_view(output)['x'].ravel()[[1, 4]], [.8, 1.])
        # Every non-coordinate byte, including row padding and acquisition
        # offsets, stays unchanged.
        mutable = set()
        for index in [1, 4]:
            row, column = divmod(index, 3)
            start = row * source.row_step + column * source.point_step
            mutable.update(range(start, start + 12))
        assert all(a == b for i, (a, b) in enumerate(
            zip(source.data, output.data)) if i not in mutable)
        assert np.isnan(cloud_view(only)['x'].ravel()[~changed]).all()
        bad = copy.deepcopy(source)
        bad.header.frame_id = 'map'
        try:
            corrected_message(bad, model)
        except ValueError:
            pass
        else:
            raise AssertionError('Frame mismatch was accepted')
        bad = copy.deepcopy(source)
        bad.row_step -= 30
        try:
            cloud_view(bad)
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid point layout was accepted')

    limits = dict(distance_error_p95_m=.05, minimum_direction_coverage=.8,
                  minimum_point_retention_fraction=.95, minimum_object_frame_fraction=.95)
    values = dict(role='review', corrected_only_error_m_p50_p95=[.01, .03],
                  direction_coverage=1., frames=10, frames_with_target_before=10,
                  frames_with_target_after=10, minimum_target_point_retention=1.,
                  floor_false_points_before_p50_p95=[5, 8],
                  floor_false_points_after_p50_p95=[0, 1])
    objects = dict(values, role='object')
    assert all(evaluation_checks(
        {'wall': values, 'object': objects}, limits).values())
    values['minimum_target_point_retention'] = .5
    assert not evaluation_checks({'wall': values, 'object': objects}, limits)[
        'target_retention']


def check_ros():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    with tempfile.TemporaryDirectory() as temp:
        path = Path(temp) / 'model.npz'
        candidate().save(path)
        rclpy.init()
        preview = Preview(SimpleNamespace(model=path, capture=None,
                                          input_topic='/odin_window_test/raw', rate=10., once=False))
        observer = rclpy.create_node('window_preview_test')
        received = {}
        source = observer.create_publisher(
            PointCloud2, '/odin_window_test/raw', qos_profile_sensor_data)
        subscriptions = [observer.create_subscription(PointCloud2, topic,
                                                      lambda msg, key=key: received.__setitem__(key, msg), qos_profile_sensor_data)
                         for key, topic in [('original', '/odin_window/preview/original'),
                                            ('corrected', '/odin_window/preview/cloud')]]
        subscriptions.append(observer.create_subscription(String, '/odin_window/preview/status',
                                                          lambda msg: received.__setitem__('status', json.loads(msg.data)), 2))
        executor = SingleThreadedExecutor()
        executor.add_node(preview)
        executor.add_node(observer)
        start = time.monotonic()
        try:
            while len(received) < 3 and time.monotonic() - start < 8:
                source.publish(message())
                executor.spin_once(timeout_sec=.05)
            assert len(received) == 3, received.keys()
            assert received['status']['corrected_points'] == 2
            assert received['status']['navigation_connected'] is False
            assert received['original'].header == received['corrected'].header
            assert np.allclose(cloud_view(received['corrected'])[
                               'x'].ravel()[[1, 4]], [.8, 1.])
            publishers = observer.get_publisher_names_and_types_by_node(
                'odin_window_preview', '')
            assert all(topic.startswith('/odin_window/preview/') or topic in ('/rosout', '/parameter_events')
                       for topic, _ in publishers)
        finally:
            executor.shutdown()
            preview.destroy_node()
            observer.destroy_node()
            rclpy.try_shutdown()


def check_capture_and_replay():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    scripts = Path(__file__).resolve().parents[1] / 'scripts'
    with tempfile.TemporaryDirectory() as temp:
        temp = Path(temp)
        capture = temp / 'capture'
        calib = temp / 'calib.yaml'
        calib.write_text('cam_0: {image_width: 4, image_height: 3}\n')
        rclpy.init()
        node = rclpy.create_node('window_capture_test')
        raw = node.create_publisher(
            PointCloud2, '/odin1/cloud_raw', qos_profile_sensor_data)
        odom = node.create_publisher(
            Odometry, '/odin1/odometry', qos_profile_sensor_data)
        wiwc = node.create_publisher(
            Odometry, '/odin1/wiwc', qos_profile_sensor_data)
        camera = node.create_publisher(
            Image, '/odin1/image', qos_profile_sensor_data)
        process = subprocess.Popen([sys.executable, str(scripts / 'capture_window_calibration.py'),
                                    '--output', str(capture), '--calib', str(calib), '--distance-cm', '100', '--seconds', '4'],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        start = time.monotonic()
        try:
            while process.poll() is None and time.monotonic() - start < 9:
                stamp = node.get_clock().now().to_msg()
                cloud = message()
                cloud.header.stamp = stamp
                raw.publish(cloud)
                pose = Odometry()
                pose.header.stamp = stamp
                pose.pose.pose.orientation.w = 1.
                odom.publish(pose)
                pose.pose.covariance[:16] = np.eye(4).ravel().tolist()
                pose.twist.covariance[:16] = np.eye(4).ravel().tolist()
                wiwc.publish(pose)
                image = Image()
                image.header.stamp = stamp
                image.width = 4
                image.height = 3
                image.step = 12
                image.encoding = 'rgb8'
                image.data = bytes(range(36))
                camera.publish(image)
                rclpy.spin_once(node, timeout_sec=.04)
            assert process.poll() == 0, process.poll()
            report = json.loads((capture / 'measurement.json').read_text())
            assert report['data_available'] and report['motion_commands_published'] is False
            with np.load(capture / 'capture.npz') as data:
                assert len(data['raw_times']) >= 10
                assert np.all(data['raw_0']['confidence'] == 100)
                assert data['extrinsics'].shape[1] == 33
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5)
            node.destroy_node()
            rclpy.try_shutdown()
        model = temp / 'model.npz'
        candidate().save(model)
        replay = subprocess.run([sys.executable, str(scripts / 'preview_window_calibration.py'),
                                 '--model', str(model), '--capture', str(capture), '--rate', '30', '--once'],
                                capture_output=True, text=True, timeout=8)
        assert replay.returncode == 0, replay.stdout + replay.stderr


if __name__ == '__main__':
    if '--ros' in sys.argv:
        check_ros()
        check_capture_and_replay()
    else:
        check_math()
        check_messages()
    print('窗口校正检查通过')
