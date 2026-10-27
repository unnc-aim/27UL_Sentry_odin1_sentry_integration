#!/usr/bin/python3 -s
"""Record existing filtered raw geometry in the native map frame for static navigation."""
import argparse
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException
from capture_color_map import decode, save
from color_map_display import ColorVoxels
from native_math import matrix, fresh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--native-map", required=True, type=Path)
    parser.add_argument("--seconds", type=float, default=180.0)
    parser.add_argument("--voxel", type=float, default=0.04)
    parser.add_argument("--max-points", type=int, default=1000000)
    args, ros = parser.parse_known_args()
    path = args.output.expanduser().resolve()
    native = args.native_map.expanduser().resolve()
    if path.exists() or path.with_suffix(".json").exists():
        parser.error("Choose a new output filename")
    if not native.is_file():
        parser.error("Native localization map file required")
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("Positive recording duration required")
    voxels = ColorVoxels(args.voxel, args.max_points)
    rclpy.init(args=ros)
    node = rclpy.create_node("record_filtered_navigation_cloud")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    seed = None
    frames = 0
    skipped = 0
    last_stamp = 0.0
    latest = None
    clock_fault = False

    def receive(msg):
        nonlocal latest, last_stamp, clock_fault
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp < last_stamp:
            clock_fault = True
            return
        last_stamp = stamp
        latest = msg

    node.create_subscription(
        PointCloud2, "/odin_obstacles/registered_scan", receive, qos_profile_sensor_data
    )

    def process():
        nonlocal latest, frames, skipped, seed
        msg = latest
        latest = None
        if msg is None or clock_fault:
            return
        now = node.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg.header.frame_id != "odom" or not fresh(stamp, now, 0.3):
            skipped += 1
            return
        try:
            current = buffer.lookup_transform("map", "odom", Time())
            ts = current.header.stamp.sec + current.header.stamp.nanosec * 1e-9
            if not fresh(ts, now, 0.3):
                skipped += 1
                return
            T = matrix(
                buffer.lookup_transform(
                    "map", "odom", Time.from_msg(msg.header.stamp)
                ).transform
            )
            if seed is None:
                base = buffer.lookup_transform("map", "base_footprint", Time())
                base_stamp = base.header.stamp.sec + base.header.stamp.nanosec * 1e-9
                if not fresh(base_stamp, now, 0.5):
                    skipped += 1
                    return
                seed = matrix(base.transform)[:3, 3].tolist()
            xyz, unused = decode(msg, require_color=False)
            voxels.add(
                xyz @ T[:3, :3].T + T[:3, 3], np.zeros(len(xyz), dtype=np.uint32)
            )
            frames += 1
        except (TransformException, ValueError) as e:
            skipped += 1
            node.get_logger().warning(str(e), throttle_duration_sec=3.0)

    node.create_timer(0.1, process)
    end = time.monotonic() + args.seconds
    try:
        while (
            rclpy.ok()
            and time.monotonic() < end
            and not clock_fault
            and not voxels.full
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if clock_fault:
        raise RuntimeError(
            "Input clock changed; repeat the recording after localization settles"
        )
    if not frames or not voxels.cells:
        raise RuntimeError("No filtered raw geometry with fresh native map TF received")
    cloud = voxels.cloud(__import__("builtin_interfaces.msg", fromlist=["Time"]).Time())
    xyz, rgb = decode(cloud)
    save(path, xyz, rgb)
    metadata = {
        "source_topic": "/odin_obstacles/registered_scan",
        "frame": "map",
        "geometry_only": True,
        "seed_ground_xyz": seed,
        "native_map": str(native),
        "source_frames": frames,
        "skipped_frames": skipped,
        "points": len(xyz),
        "voxel_m": args.voxel,
        "capacity_reached": voxels.full,
        "note": "Only observed regions are captured. Native BIN remains unchanged.",
    }
    path.with_suffix(".json").write_text(json.dumps(metadata, indent=2))
    print(json.dumps(metadata, indent=2))
    print("Saved filtered navigation observations:", path)


if __name__ == "__main__":
    main()
