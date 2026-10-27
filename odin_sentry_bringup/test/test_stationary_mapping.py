"""Compare fixed-chassis mapping with changing scan coverage, in Domain 90."""
import math
import os
import runpy
import signal
import subprocess
import tempfile
import time
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "90"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
import yaml
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster


def main():
    share = Path(get_package_share_directory("odin_sentry_bringup"))
    source = Path(get_package_share_directory("pb2025_nav_bringup")) / (
        "config/reality/nav2_params.yaml"
    )
    launch = runpy.run_path(str(share / "launch/odin_sentry.launch.py"))
    updated = launch["navigation_parameters"](source, 0.3, 0.5)
    configs = {
        "original": yaml.safe_load(source.read_text()),
        "updated": updated,
    }
    executable = str(
        Path(get_package_prefix("slam_toolbox"))
        / "lib/slam_toolbox/sync_slam_toolbox_node"
    )
    processes, logs = [], []
    rclpy.init()
    node = rclpy.create_node("odin_stationary_mapping_test")
    static = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_footprint"
    transform.transform.rotation.w = 1.0
    static.sendTransform(transform)
    maps, publishers, subscriptions = {}, {}, []
    with tempfile.TemporaryDirectory(prefix="odin_mapping_test_") as directory:
        try:
            for name, config in configs.items():
                params = dict(config["slam_toolbox"]["ros__parameters"])
                params.update(
                    scan_topic=f"/{name}/scan",
                    map_name=f"/{name}/map",
                    transform_publish_period=0.0,
                    enable_interactive_mode=False,
                )
                path = Path(directory) / f"{name}.yaml"
                path.write_text(
                    yaml.safe_dump({"/**": {"ros__parameters": params}})
                )
                log = open(Path(directory) / f"{name}.log", "w+")
                logs.append(log)
                processes.append(
                    subprocess.Popen(
                        [
                            executable,
                            "--ros-args",
                            "-r",
                            f"__ns:=/{name}",
                            "--params-file",
                            str(path),
                        ],
                        stdout=log,
                        stderr=subprocess.STDOUT,
                    )
                )
                publishers[name] = node.create_publisher(
                    LaserScan, f"/{name}/scan", 5
                )
                subscriptions.append(
                    node.create_subscription(
                        OccupancyGrid,
                        f"/{name}/map",
                        lambda msg, name=name: maps.update({name: msg}),
                        5,
                    )
                )

            def scan_for(seconds, direction):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    assert all(
                        p.poll() is None for p in processes
                    ), "Mapping process exited"
                    scan = LaserScan()
                    scan.header.stamp = node.get_clock().now().to_msg()
                    scan.header.frame_id = "base_footprint"
                    scan.angle_min = -math.pi
                    scan.angle_increment = math.pi / 360
                    scan.angle_max = (
                        scan.angle_min + 719 * scan.angle_increment
                    )
                    scan.range_min, scan.range_max, scan.scan_time = (
                        0.3,
                        10.0,
                        0.1,
                    )
                    scan.ranges = [
                        3.0
                        if direction
                        * math.cos(scan.angle_min + i * scan.angle_increment)
                        > math.cos(0.5)
                        else math.inf
                        for i in range(720)
                    ]
                    for publisher in publishers.values():
                        publisher.publish(scan)
                    until = time.monotonic() + 0.1
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=0.01)

            def occupied_left(msg):
                return sum(
                    value > 50
                    and msg.info.origin.position.x
                    + (i % msg.info.width + 0.5) * msg.info.resolution
                    < -2.0
                    for i, value in enumerate(msg.data)
                )

            scan_for(4.0, 1)
            assert set(maps) == set(configs), "Initial maps missing"
            assert all(occupied_left(m) == 0 for m in maps.values())
            scan_for(3.0, -1)
            counts = {name: occupied_left(msg) for name, msg in maps.items()}
            assert counts["original"] == 0, counts
            assert counts["updated"] > 10, counts
            print(
                f"PASS: fixed chassis, new opposite-view occupied cells: {counts}"
            )
        except BaseException:
            for log in logs:
                log.flush()
                log.seek(0)
                print(log.read()[-3000:])
            raise
        finally:
            for process in processes:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            for log in logs:
                log.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
