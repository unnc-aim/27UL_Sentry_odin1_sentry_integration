#!/usr/bin/env python3
"""Bounded persistent RGB display in device odometry coordinates."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from capture_color_map import decode


class ColorVoxels:
    def __init__(self, resolution=0.10, limit=500000):
        if not math.isfinite(resolution) or resolution <= 0 or limit < 1:
            raise ValueError("Positive voxel size and point count required")
        self.resolution, self.limit = resolution, limit
        self.cells = {}
        self.full = False

    def add(self, points, colors):
        keys = np.floor(points / self.resolution).astype(np.int64)
        _, selected = np.unique(keys, axis=0, return_index=True)
        for i in selected:
            key = tuple(keys[i])
            if key in self.cells or len(self.cells) < self.limit:
                self.cells[key] = (
                    float(points[i, 0]),
                    float(points[i, 1]),
                    float(points[i, 2]),
                    int(colors[i]),
                )
            else:
                self.full = True

    def cloud(self, stamp):
        dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
        values = np.array(list(self.cells.values()), dtype=dtype)
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = "odin_local"
        msg.height = 1
        msg.width = len(values)
        msg.point_step = 16
        msg.row_step = 16 * len(values)
        msg.is_dense = True
        msg.fields = [
            PointField(
                name=k,
                offset=4 * i,
                datatype=PointField.FLOAT32 if i < 3 else PointField.UINT32,
                count=1,
            )
            for i, k in enumerate(("x", "y", "z", "rgb"))
        ]
        msg.data = values.tobytes()
        return msg


class Display(Node):
    def __init__(self):
        super().__init__("odin_color_map_display")
        self.voxels = ColorVoxels(
            float(self.declare_parameter("voxel_size", 0.10).value),
            int(self.declare_parameter("max_points", 500000).value),
        )
        self.latest = None
        self.last_stamp = 0.0
        self.fault = False
        self.pub = self.create_publisher(
            PointCloud2,
            "/odin_3d/color_map",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            PointCloud2,
            "/odin_3d/color_filtered",
            self.receive,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self.update)
        self.create_timer(1.0, self.publish)

    def receive(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp < self.last_stamp:
            self.fault = True
            self.get_logger().error(
                "Cloud clock reset; restart the display to begin another map"
            )
        if msg.header.frame_id == "odin_local" and not self.fault:
            self.last_stamp = stamp
            self.latest = msg

    def update(self):
        msg = self.latest
        self.latest = None
        if msg is None:
            return
        age = (self.get_clock().now().nanoseconds * 1e-9) - (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )
        if not -0.05 <= age <= 0.5:
            return
        try:
            self.voxels.add(*decode(msg))
            if self.voxels.full:
                self.get_logger().warning(
                    "Display point limit reached; native device mapping continues",
                    throttle_duration_sec=10.0,
                )
        except (ValueError, TypeError) as e:
            self.get_logger().warning(str(e), throttle_duration_sec=3.0)

    def publish(self):
        if self.voxels.cells:
            self.pub.publish(self.voxels.cloud(self.get_clock().now().to_msg()))


def main():
    rclpy.init()
    node = Display()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
