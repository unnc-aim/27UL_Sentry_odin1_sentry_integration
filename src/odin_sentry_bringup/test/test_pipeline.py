"""Exercise the installed Sentry transforms with synthetic Odin data."""
import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

# Keep synthetic odometry and TF in a separate ROS domain.
os.environ["ROS_DOMAIN_ID"] = "87"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
import yaml
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def main():
    processes = []
    config = yaml.safe_load(
        (
            Path(get_package_share_directory("odin_sentry_bringup"))
            / "config/self_filter.yaml"
        ).read_text()
    )["odin_nav_adapter"]["ros__parameters"]
    radius = config["self_filter_radius"]
    body_points = [[0.0, 0.0, -radius * 0.2], [radius - 0.01, 0.0, 0.0]]
    outside_points = []
    for axis in range(3):
        for sign in (-1, 1):
            point = [0.0, 0.0, 0.0]
            point[axis] = sign * (radius + 0.01)
            outside_points.append(point)
    lower, upper = config["self_filter_box_min"], config["self_filter_box_max"]
    center = [(a + b) / 2 for a, b in zip(lower, upper)]
    body_points += [center, [lower[0] + 0.01, upper[1] - 0.01, center[2]]]
    for axis in range(3):
        for bound, margin in ((lower, -0.01), (upper, 0.01)):
            point = center.copy()
            point[axis] = bound[axis] + margin
            outside_points.append(point)
    filtered_size = 2 + len(outside_points)

    def world_points(points, theta):
        yaw = 0.4 + theta
        return [
            (
                2.0 + (x + 0.13) * math.cos(yaw) - (y + 0.07) * math.sin(yaw),
                3.0 + (x + 0.13) * math.sin(yaw) + (y + 0.07) * math.cos(yaw),
                z + 0.43,
                1.0,
            )
            for x, y, z in points
        ]

    with tempfile.TemporaryDirectory(
        prefix="odin_pipeline_test_"
    ) as directory:
        logs = []
        try:
            commands = [
                [sys.argv[1]],
                [
                    str(
                        Path(get_package_prefix("loam_interface"))
                        / "lib/loam_interface/loam_interface_node"
                    ),
                    "--ros-args",
                    "-p",
                    "state_estimation_topic:=odin_navigation/odometry",
                    "-p",
                    "registered_scan_topic:=odin_navigation/cloud",
                    "-p",
                    "base_frame:=base_footprint",
                    "-p",
                    "lidar_frame:=odin_imu",
                    "-p",
                    "align_initial_pose:=true",
                ],
                [
                    str(
                        Path(get_package_prefix("sensor_scan_generation"))
                        / "lib/sensor_scan_generation/sensor_scan_generation_node"
                    ),
                    "--ros-args",
                    "-p",
                    "base_frame:=base_footprint",
                    "-p",
                    "robot_base_frame:=gimbal_yaw",
                    "-p",
                    "sync_min_interval:=0.01",
                    "-p",
                    "lidar_frame:=odin_imu",
                ],
            ]
            filtered_command = [
                sys.argv[1],
                "--ros-args",
                "-r",
                "__node:=odin_filtered_test",
                "-r",
                "odin_navigation/cloud:=filtered/cloud",
                "-r",
                "odin_navigation/odometry:=filtered/odometry",
            ]
            for name, value in config.items():
                filtered_command += ["-p", f"{name}:={value}"]
            commands.append(
                filtered_command + ["-p", "near_outlier_range:=0.0"]
            )
            for index, command in enumerate(commands):
                log = open(Path(directory) / f"node_{index}.log", "w+")
                logs.append(log)
                processes.append(
                    subprocess.Popen(
                        command, stdout=log, stderr=subprocess.STDOUT
                    )
                )
            rclpy.init()
            node = rclpy.create_node("odin_pipeline_test")
            static = StaticTransformBroadcaster(node)
            dynamic = TransformBroadcaster(node)
            fixed = TransformStamped()
            fixed.header.frame_id = "gimbal_yaw"
            fixed.child_frame_id = "odin_mount"
            fixed.transform.translation.x = 0.13
            fixed.transform.translation.y = 0.07
            fixed.transform.translation.z = 0.327
            fixed.transform.rotation.w = 1.0
            imu = TransformStamped()
            imu.header.frame_id = "odin_mount"
            imu.child_frame_id = "odin_imu"
            imu.transform.rotation.w = 1.0
            pose_pub = node.create_publisher(Odometry, "odin1/odometry", 5)
            cloud_pub = node.create_publisher(
                PointCloud2, "odin1/cloud_slam", 5
            )
            received = {
                "adapted": [],
                "cloud": [],
                "body": [],
                "registered": [],
                "filtered": [],
            }
            subscriptions = []
            for key, topic, message in (
                ("adapted", "odin_navigation/odometry", Odometry),
                ("cloud", "odin_navigation/cloud", PointCloud2),
                ("body", "odometry", Odometry),
                ("registered", "registered_scan", PointCloud2),
                ("filtered", "filtered/cloud", PointCloud2),
            ):
                subscriptions.append(
                    node.create_subscription(
                        message,
                        topic,
                        lambda msg, key=key: received[key].append(msg),
                        10,
                    )
                )

            def publish(
                theta=0.0,
                age=0.0,
                jump=0.0,
                publish_tf=True,
                cloud_offset=0.0,
                tf_offset=0.0,
                body_only=False,
                points=None,
            ):
                stamp = (
                    node.get_clock().now()
                    - rclpy.duration.Duration(seconds=age)
                ).to_msg()
                transform = TransformStamped()
                transform.header = Header(
                    stamp=(
                        rclpy.time.Time.from_msg(stamp)
                        + rclpy.duration.Duration(seconds=tf_offset)
                    ).to_msg(),
                    frame_id="base_footprint",
                )
                transform.child_frame_id = "gimbal_yaw"
                transform.transform.translation.z = 0.103
                transform.transform.rotation.z = math.sin(theta / 2)
                transform.transform.rotation.w = math.cos(theta / 2)
                if publish_tf:
                    dynamic.sendTransform(transform)
                # A fixed chassis in an arbitrary Odin world, with a rotating yaw mount.
                yaw = 0.4 + theta
                pose = Odometry()
                pose.header = Header(stamp=stamp, frame_id="odom")
                pose.child_frame_id = "imu"
                pose.pose.pose.position.x = (
                    2.0 + 0.13 * math.cos(yaw) - 0.07 * math.sin(yaw) + jump
                )
                pose.pose.pose.position.y = (
                    3.0 + 0.13 * math.sin(yaw) + 0.07 * math.cos(yaw)
                )
                pose.pose.pose.position.z = 0.43
                pose.pose.pose.orientation.z = math.sin(yaw / 2)
                pose.pose.pose.orientation.w = math.cos(yaw / 2)
                fields = [
                    PointField(
                        name=name,
                        offset=4 * i,
                        datatype=PointField.FLOAT32,
                        count=1,
                    )
                    for i, name in enumerate(("x", "y", "z", "rgb"))
                ]
                cloud_stamp = (
                    rclpy.time.Time.from_msg(stamp)
                    + rclpy.duration.Duration(seconds=cloud_offset)
                ).to_msg()
                cloud = point_cloud2.create_cloud(
                    Header(stamp=cloud_stamp, frame_id="odom"),
                    fields,
                    world_points(points, theta)
                    if points is not None
                    else world_points(body_points, theta)
                    if body_only
                    else [
                        (3.0, 3.0, 0.0, 1.0),
                        (4.0, 2.0, 0.5, 2.0),
                        (math.nan, 0.0, 0.0, 0.0),
                    ]
                    + world_points(body_points + outside_points, theta),
                )
                pose_pub.publish(pose)
                cloud_pub.publish(cloud)
                return stamp

            def run_for(seconds, **kwargs):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    assert all(
                        p.poll() is None for p in processes
                    ), "A pipeline process exited"
                    publish(**kwargs)
                    until = time.monotonic() + 0.05
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=0.005)

            run_for(1.5, age=5.0, publish_tf=False)
            assert not received[
                "adapted"
            ], "Old device-clock data reached navigation"
            # The optional odin_local TF anchor can arrive after fresh inputs.
            first_stamp = publish(publish_tf=False)
            end = time.monotonic() + 0.08
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.005)
            assert any(
                msg.header.stamp == first_stamp for msg in received["adapted"]
            ), "Fresh input waited for the optional TF anchor"
            # TF discovery can take longer than the input timeout at startup.
            run_for(0.8, publish_tf=False)
            assert (
                received["filtered"][-1].width == filtered_size
            ), "Near-origin filtering waited for mount TF"
            static.sendTransform([fixed, imu])
            run_for(2.0, theta=0.5)
            assert (
                received["adapted"]
                and received["body"]
                and received["registered"]
            )
            first = received["adapted"][0]
            assert (
                abs(
                    first.pose.pose.position.x
                    - (2.0 + 0.13 * math.cos(0.4) - 0.07 * math.sin(0.4))
                )
                < 1e-6
            )
            assert abs(first.pose.pose.position.z - 0.43) < 1e-6
            assert abs(first.pose.pose.orientation.w - math.cos(0.2)) < 1e-6
            first_twist = received["body"][0].twist.twist
            assert first_twist.linear.x == 0.0 and first_twist.angular.z == 0.0
            cloud = received["cloud"][-1]
            assert cloud.width == filtered_size + len(body_points) and [
                f.name for f in cloud.fields
            ] == [
                "x",
                "y",
                "z",
                "intensity",
            ]
            assert struct.unpack_from("<f", cloud.data, 12)[0] == 0.0
            expected = (math.cos(0.4), -math.sin(0.4), 0.0)
            point = tuple(
                point_cloud2.read_points(
                    received["registered"][-1], field_names=("x", "y", "z")
                )
            )[0]
            assert all(
                abs(float(v) - e) < 1e-5 for v, e in zip(point, expected)
            ), (point, expected)
            # PTP correction can change slightly between the two driver callbacks.
            for offset in (0.000035, 0.0007, -0.0007):
                expected_stamp = publish(
                    theta=0.5, cloud_offset=offset, tf_offset=0.001
                )
                end = time.monotonic() + 0.08
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.005)
                assert any(
                    msg.header.stamp == expected_stamp
                    for msg in received["adapted"]
                ), "A near-equal timestamp pair waited for another sensor frame"
                cloud_stamp = (
                    rclpy.time.Time.from_msg(expected_stamp)
                    + rclpy.duration.Duration(seconds=offset)
                ).to_msg()
                assert any(
                    msg.header.stamp == cloud_stamp for msg in received["body"]
                ), "Chassis odometry waited for another sensor frame"
            run_for(1.0, theta=1.0)
            body = received["body"][-1]
            assert body.child_frame_id == "gimbal_yaw"
            assert (
                abs(body.pose.pose.position.x) < 1e-5
                and abs(body.pose.pose.position.y) < 1e-5
            )
            assert abs(body.pose.pose.position.z - 0.103) < 1e-5
            assert abs(body.pose.pose.orientation.z - math.sin(0.5)) < 1e-5
            point = tuple(
                point_cloud2.read_points(
                    received["registered"][-1], field_names=("x", "y", "z")
                )
            )[0]
            assert all(
                abs(float(v) - e) < 1e-5 for v, e in zip(point, expected)
            )
            filtered = received["filtered"][-1]
            points = list(
                point_cloud2.read_points(filtered, field_names=("x", "y", "z"))
            )
            expected_points = [(3.0, 3.0, 0.0), (4.0, 2.0, 0.5)] + [
                p[:3] for p in world_points(outside_points, 1.0)
            ]
            assert len(points) == len(
                expected_points
            ), "Body filtering removed nearby outside points"
            assert all(
                abs(float(v) - e) < 1e-5
                for p, expected_point in zip(points, expected_points)
                for v, e in zip(p, expected_point)
            ), "Platform filter changed world coordinates or failed to follow the gimbal"
            run_for(0.7, theta=1.0, body_only=True)
            assert received["filtered"][-1].width == 0
            run_for(0.2, theta=1.0)
            assert (
                received["filtered"][-1].width == filtered_size
            ), "All-body frames interrupted pose continuity"
            before_delayed = received["adapted"][-1].header.stamp
            run_for(0.08, theta=1.0, age=1.0, publish_tf=False)
            run_for(0.5, theta=1.0)
            assert received["adapted"][-1].header.stamp != before_delayed
            last_stamp = received["adapted"][-1].header.stamp
            run_for(0.8, theta=1.0, jump=2.0)
            run_for(0.8, theta=1.0)
            assert (
                received["adapted"][-1].header.stamp == last_stamp
            ), "Discontinuity resumed the previous navigation origin"
            processes[0].send_signal(signal.SIGINT)
            processes[0].wait(timeout=5)
            processes[0] = subprocess.Popen(
                commands[0], stdout=logs[0], stderr=subprocess.STDOUT
            )
            run_for(1.5, theta=1.0)
            assert received["adapted"][-1].header.stamp != last_stamp
            end = time.monotonic() + 0.8
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            last_stamp = received["adapted"][-1].header.stamp
            run_for(0.8, theta=1.0)
            assert received["adapted"][-1].header.stamp == last_stamp

            # Use the configured near-field filter with a small solid obstacle,
            # an isolated near return, and an isolated distant return.
            processes[3].send_signal(signal.SIGINT)
            processes[3].wait(timeout=5)
            processes[3] = subprocess.Popen(
                filtered_command, stdout=logs[3], stderr=subprocess.STDOUT
            )
            dense = [
                [0.5 + x * 0.005, -0.3 + y * 0.005, 0.1 + z * 0.005]
                for x in range(2)
                for y in range(2)
                for z in range(2)
            ]
            isolated = [0.7, 0.3, 0.0]
            distant = [config["near_outlier_range"] + 0.4, 0.0, 0.0]
            for theta, seconds in ((0.5, 1.0), (1.0, 0.3)):
                run_for(
                    seconds, theta=theta, points=dense + [isolated, distant]
                )
                points = list(
                    point_cloud2.read_points(
                        received["filtered"][-1], field_names=("x", "y", "z")
                    )
                )
                expected_points = world_points(dense + [distant], theta)
                assert len(points) == len(
                    expected_points
                ), "Near outlier filter removed a supported obstacle or retained an isolated return"
                assert all(
                    abs(float(v) - e) < 1e-5
                    for p, expected_point in zip(points, expected_points)
                    for v, e in zip(p, expected_point[:3])
                ), "Near outlier filter changed retained world coordinates"
            run_for(0.2, theta=1.0, points=[isolated])
            assert received["filtered"][-1].width == 0
            run_for(0.2, theta=1.0, points=dense)
            assert received["filtered"][-1].width == len(dense)
            print(
                "PASS: clock filtering, first pose, XYZI, rotating mount, stationary chassis, body filtering, near outliers and discontinuity stop"
            )
            node.destroy_node()
            rclpy.shutdown()
        except BaseException:
            for log in logs:
                log.flush()
                log.seek(0)
                print(log.read()[-4000:])
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


if __name__ == "__main__":
    main()
