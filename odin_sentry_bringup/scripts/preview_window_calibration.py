#!/usr/bin/env python3
"""Publish optical-window correction on independent preview topics."""
import argparse
import copy
import json
import re
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from window_calibration_core import DistanceModel, cloud_view


def corrected_message(message, model):
    if message.header.frame_id != model.metadata['frame_id']:
        raise ValueError(
            'Cloud frame differs from the calibration sensor frame')
    source = cloud_view(message)
    xyz = np.column_stack([source[k].ravel() for k in ('x', 'y', 'z')])
    result, changed = model.apply(xyz, source['confidence'].ravel())
    output = copy.deepcopy(message)
    view = cloud_view(output)
    for index, name in enumerate(('x', 'y', 'z')):
        view[name][:] = result[:, index].reshape(source.shape)
    only = copy.deepcopy(output)
    view = cloud_view(only)
    for name in ('x', 'y', 'z'):
        view[name][~changed.reshape(source.shape)] = np.nan
    only.is_dense = False
    return output, only, changed


def replay_message(array, stamp, frame_id):
    array = np.ascontiguousarray(array.reshape(-1))
    fields = []
    types = {('f', 4): PointField.FLOAT32, ('u', 1): PointField.UINT8,
             ('u', 2): PointField.UINT16, ('u', 4): PointField.UINT32}
    for name in array.dtype.names:
        dtype, offset = array.dtype.fields[name][:2]
        fields.append(PointField(name=name, offset=offset,
                      datatype=types[(dtype.kind, dtype.itemsize)], count=1))
    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(array)
    message.fields = fields
    message.is_bigendian = False
    message.point_step = array.dtype.itemsize
    message.row_step = message.width * message.point_step
    message.data = array.tobytes()
    message.is_dense = False
    return message


class Preview(Node):
    def __init__(self, args):
        super().__init__('odin_window_preview')
        self.model = DistanceModel.load(args.model)
        self.args = args
        self.last = 0.
        self.index = 0
        self.finished = False
        self.original = self.create_publisher(
            PointCloud2, '/odin_window/preview/original', qos_profile_sensor_data)
        self.corrected = self.create_publisher(
            PointCloud2, '/odin_window/preview/cloud', qos_profile_sensor_data)
        self.only = self.create_publisher(
            PointCloud2, '/odin_window/preview/corrected_only', qos_profile_sensor_data)
        self.status = self.create_publisher(
            String, '/odin_window/preview/status', 2)
        self.data = None
        if args.capture:
            self.data = np.load(
                args.capture / 'capture.npz', allow_pickle=False)
            self.keys = sorted((k for k in self.data.files if re.fullmatch(
                r'raw_\d+', k)), key=lambda k: int(k[4:]))
            if not self.keys:
                raise ValueError('Capture has no raw frames')
            self.timer = self.create_timer(1 / args.rate, self.replay)
        else:
            self.subscription = self.create_subscription(
                PointCloud2, args.input_topic, self.receive, qos_profile_sensor_data)

    def replay(self):
        message = replay_message(self.data[self.keys[self.index]], self.get_clock(
        ).now().to_msg(), self.model.metadata['frame_id'])
        self.receive(message, replay=True)
        self.index += 1
        if self.index == len(self.keys):
            if self.args.once:
                self.timer.cancel()
                self.finished = True
            else:
                self.index = 0

    def receive(self, message, replay=False):
        now = time.monotonic()
        if not replay and now - self.last < 1 / self.args.rate:
            return
        self.last = now
        try:
            start = time.perf_counter()
            result, only, changed = corrected_message(message, self.model)
            elapsed = (time.perf_counter() - start) * 1000
            self.original.publish(message)
            self.corrected.publish(result)
            self.only.publish(only)
            self.status.publish(String(data=json.dumps({'mode': 'preview', 'replay': replay,
                                                        'points': len(changed), 'corrected_points': int(changed.sum()),
                                                        'processing_ms': elapsed, 'navigation_connected': False,
                                                        'field_verified': self.model.metadata['field_verified']})))
        except ValueError as error:
            self.status.publish(String(data=json.dumps(
                {'mode': 'preview', 'error': str(error)})))
            self.get_logger().warning(str(error))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--capture', type=Path)
    parser.add_argument('--input-topic', default='/odin1/cloud_raw')
    parser.add_argument('--rate', type=float, default=10.)
    parser.add_argument('--once', action='store_true')
    args, ros_args = parser.parse_known_args()
    if not 0 < args.rate <= 30:
        parser.error('rate must be between 0 and 30 Hz')
    rclpy.init(args=ros_args)
    node = None
    try:
        node = Preview(args)
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            if node.data is not None:
                node.data.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
