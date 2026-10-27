#!/usr/bin/env python3
"""Record a stationary measured surface without publishing motion commands."""
import argparse
import json
import shutil
import time
from pathlib import Path

import numpy as np
import rclpy
from chassis_controllers.msg import ChassisControl
from nav_msgs.msg import Odometry
from PIL import Image as PILImage
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from window_calibration_core import cloud_view


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--distance-cm', type=float, required=True)
    parser.add_argument('--surface', default='matte_wall')
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--calib', type=Path,
                        default=Path.home() / '.ros/odin_ros_driver/calib.yaml')
    args = parser.parse_args()
    if args.output.exists() or not args.calib.is_file():
        parser.error(
            'Use a new output directory and an existing calibration file')
    if not 10 <= args.distance_cm <= 400 or not 1 <= args.seconds <= 30:
        parser.error('Use 10–400 cm and 1–30 seconds')
    args.output.mkdir(parents=True)
    raw, poses, extrinsics, commands, images = [], [], [], [], []
    last = {'raw': 0., 'image': 0.}
    rclpy.init()
    node = rclpy.create_node('odin_window_capture')

    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def cloud(message):
        now = time.monotonic()
        if now - last['raw'] >= .19:
            last['raw'] = now
            raw.append((stamp(message), cloud_view(
                message).reshape(-1).copy()))

    def pose(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        poses.append([stamp(message), p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def calibration(message):
        extrinsics.append(
            [stamp(message), *message.twist.covariance[:16], *message.pose.covariance[:16]])

    def camera(message):
        now = time.monotonic()
        if now - last['image'] >= 3:
            last['image'] = now
            images.append(message)

    subscriptions = [
        node.create_subscription(
            PointCloud2, '/odin1/cloud_raw', cloud, qos_profile_sensor_data),
        node.create_subscription(
            Odometry, '/odin1/odometry', pose, qos_profile_sensor_data),
        node.create_subscription(
            Odometry, '/odin1/wiwc', calibration, qos_profile_sensor_data),
        node.create_subscription(
            Image, '/odin1/image', camera, qos_profile_sensor_data),
        node.create_subscription(ChassisControl, '/chassis_command',
                                 lambda m: commands.append(
                                     [time.time(), m.x_speed, m.y_speed, m.spin_speed, m.emergency_stop]),
                                 qos_profile_sensor_data),
    ]
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.02)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    payload = {f'raw_{i}': array for i, (_, array) in enumerate(raw)}
    payload.update(raw_times=np.array([t for t, _ in raw]), poses=np.array(poses),
                   extrinsics=np.array(extrinsics), commands=np.array(commands))
    np.savez_compressed(args.output / 'capture.npz', **payload)
    shutil.copy2(args.calib, args.output / 'calib.yaml')
    report = {'reference_distance_cm': args.distance_cm, 'surface': args.surface,
              'reference': 'Observation face to surface; axis convention recorded in the manifest',
              'seconds': args.seconds, 'raw_frames': len(raw), 'pose_frames': len(poses),
              'calibration_frames': len(extrinsics), 'images': [], 'motion_commands_published': False}
    for index, message in enumerate(images):
        if message.encoding not in ('bgr8', 'rgb8'):
            continue
        array = np.frombuffer(message.data, np.uint8).reshape(
            message.height, message.step)
        array = array[:, :message.width *
                      3].reshape(message.height, message.width, 3)
        if message.encoding == 'bgr8':
            array = array[:, :, ::-1]
        filename = f'image_{index}.png'
        PILImage.fromarray(array).save(args.output / filename)
        report['images'].append({'file': filename, 'stamp': stamp(message)})
    if poses:
        report['position_span_m'] = np.ptp(
            np.array(poses)[:, 1:4], axis=0).tolist()
    if commands:
        report['max_command'] = np.max(
            abs(np.array(commands)[:, 1:4]), axis=0).tolist()
    report['data_available'] = len(raw) >= 10 and len(
        extrinsics) >= 5 and len(poses) >= 5 and bool(report['images'])
    (args.output / 'measurement.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    if not report['data_available']:
        raise SystemExit(
            'Capture saved; raw, poses, calibration and images are required for fitting')


if __name__ == '__main__':
    main()
