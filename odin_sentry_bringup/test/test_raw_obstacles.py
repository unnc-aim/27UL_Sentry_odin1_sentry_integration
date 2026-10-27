"""Exercise raw deskew, device calibration, alignment and missing-data handling."""
from array import array
import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "97"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage


def transform(point, yaw, offset):
    x, y, z = point
    c, s = math.cos(yaw), math.sin(yaw)
    return (c*x-s*y+offset[0], s*x+c*y+offset[1], z+offset[2])


def inverse(point, yaw, offset):
    return transform([a-b for a, b in zip(point, offset)], -yaw, (0, 0, 0))


def stamp(ns):
    return rclpy.time.Time(nanoseconds=ns).to_msg()


def main():
    max_range = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    assert 0.15 < max_range <= 30.0
    rclpy.init()
    node = rclpy.create_node("raw_obstacle_check")
    publishers = {name: node.create_publisher(kind, name, 2000 if "highfreq" in name else 20)
                  for name, kind in (("/odin1/cloud_raw", PointCloud2),
                                     ("/odin1/odometry_highfreq", Odometry),
                                     ("/odin1/wiwc", Odometry),
                                     ("/odin_navigation/odometry", Odometry),
                                     ("/lidar_odometry", Odometry))}
    outputs, odometry, sensor_transforms = [], [], []
    subscriptions = [
        node.create_subscription(PointCloud2, "/odin_obstacles/registered_scan", outputs.append, 20),
        node.create_subscription(Odometry, "/odin_obstacles/lidar_odometry", odometry.append, 20),
        node.create_subscription(TFMessage, "/tf", lambda m: sensor_transforms.extend(
            t for t in m.transforms if t.child_frame_id == "odin_obstacle_sensor"), 20),
    ]
    epoch = time.time_ns()
    alignment_yaw, alignment_xyz = 0.7, (2.0, -1.0, 0.3)
    extrinsic_yaw, extrinsic_xyz = 0.13, (-0.027, 0.019, 0.035)

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.005)

    def sensor_pose(ns):
        elapsed = (ns-epoch)*1e-9
        return (0.2+elapsed*0.7, (0.1*elapsed, -0.2, 0.43))

    def pose_message(ns, aligned=False, device=False):
        yaw, xyz = sensor_pose(ns)
        if aligned:
            yaw += alignment_yaw
            xyz = transform(xyz, alignment_yaw, alignment_xyz)
        msg = Odometry()
        msg.header.stamp = stamp(ns)
        msg.header.frame_id = "odin_local" if device else "odom"
        msg.child_frame_id = "odin_imu" if aligned or device else "imu"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = xyz
        msg.pose.pose.orientation.z = math.sin(yaw/2)
        msg.pose.pose.orientation.w = math.cos(yaw/2)
        return msg

    def reference(ns, shift=0.0):
        publishers["/odin_navigation/odometry"].publish(pose_message(ns, device=True))
        msg = pose_message(ns, aligned=True)
        msg.pose.pose.position.x += shift
        publishers["/lidar_odometry"].publish(msg)

    calibrated = False

    def calibration(valid=True, ns=None):
        nonlocal calibrated
        msg = Odometry()
        msg.header.stamp = stamp(ns if ns is not None else time.time_ns()-200_000_000)
        if valid:
            calibrated = True
            c, s = math.cos(extrinsic_yaw), math.sin(extrinsic_yaw)
            x, y, z = extrinsic_xyz
            msg.twist.covariance[:16] = [c, -s, 0., x, s, c, 0., y, 0., 0., 1., z, 0., 0., 0., 1.]
        publishers["/odin1/wiwc"].publish(msg)

    def batch(with_reference=True, gap=False, tail=True, bad=None):
        start = time.time_ns() - 140_000_000
        if calibrated:
            calibration(ns=start-2_500_000)
        expected = []
        packed = []
        # A stationary world surface observed by a moving and rotating IMU.
        for group in range(32):
            offset = struct.unpack("<f", struct.pack("<f", group*0.003))[0]
            ns = start + round(offset*1e9)
            yaw, xyz = sensor_pose(ns)
            for j in range(2):
                point = (1.4 + j*0.03, -0.3+group*0.005, 0.15+j*0.25)
                local = inverse(inverse(point, yaw, xyz), extrinsic_yaw, extrinsic_xyz)
                packed.append(struct.pack("<fffBHf", *local, 7, 900, offset))
                expected.append(transform(point, alignment_yaw, alignment_xyz))
        # A lone near return outside the body sphere must be removed before terrain accumulation.
        sparse = inverse((0.1, 0.25, -0.1), extrinsic_yaw, extrinsic_xyz)
        packed.append(struct.pack("<fffBHf", *sparse, 7, 900, 0.0))
        packed.extend([
            struct.pack("<fffBHf", 1., 0., 0., 7, 1, 0.),
            struct.pack("<fffBHf", 0., 0., 0., 7, 900, 0.),
            struct.pack("<fffBHf", max_range + 0.01, 0., 0., 7, 900, 0.),
            struct.pack("<fffBHf", 0.01, 0., 0., 7, 900, 0.),
        ])
        if max_range > 3.0:
            local = (max_range - 0.01, 0., 0.)
            packed.append(struct.pack("<fffBHf", *local, 7, 900, 0.))
            yaw, xyz = sensor_pose(start)
            point = transform(transform(local, extrinsic_yaw, extrinsic_xyz), yaw, xyz)
            expected.append(transform(point, alignment_yaw, alignment_xyz))
        cloud = PointCloud2()
        cloud.header.stamp = stamp(start)
        cloud.header.frame_id = "lidar"
        cloud.height = 1
        cloud.width = len(packed)
        cloud.fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
                        for name, offset, kind in (("x", 0, 7), ("y", 4, 7), ("z", 8, 7),
                                                   ("intensity", 12, 2), ("confidence", 13, 4),
                                                   ("offset_time", 15, 7))]
        cloud.point_step = 19
        cloud.row_step = 19*cloud.width
        cloud.data = b"".join(packed)
        if bad == "offset":
            cloud.data[15:19] = array("B", struct.pack("<f", -0.1))
        if bad == "layout":
            cloud.fields[-1].offset = 18
        if bad == "quality":
            for i in range(cloud.width):
                cloud.data[i*19+13:i*19+15] = array("B", [0, 0])
        samples = []
        for i in range(-1, 40):
            if gap and 12 <= i <= 26:
                continue
            ns = start+i*2_500_000
            msg = pose_message(ns)
            if tail or i <= 20:
                publishers["/odin1/odometry_highfreq"].publish(msg)
            else:
                samples.append(msg)
        if with_reference:
            reference(start+25_000_000)
        publishers["/odin1/cloud_raw"].publish(cloud)
        return start, expected, samples

    def expect_cloud(before, expected, start):
        until = time.monotonic()+0.25
        while len(outputs) == before and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.005)
        assert len(outputs) == before+1, (before, len(outputs))
        cloud = outputs[-1]
        points = list(point_cloud2.read_points(cloud, field_names=("x", "y", "z", "intensity")))
        assert cloud.header.frame_id == "odom" and len(points) == len(expected), len(points)
        error = max(abs(p[i]-q[i]) for p, q in zip(points, expected) for i in range(3))
        assert error < 2e-5, error
        assert all(p[3] == 0 for p in points)
        end = start+round(struct.unpack("<f", struct.pack("<f", 31*0.003))[0]*1e9)
        assert rclpy.time.Time.from_msg(cloud.header.stamp).nanoseconds == end
        spin(0.01)
        actual = odometry[-1]
        wanted = pose_message(end, aligned=True)
        assert actual.header.stamp == cloud.header.stamp
        assert actual.child_frame_id == "odin_imu"
        for axis in "xyz":
            assert abs(getattr(actual.pose.pose.position, axis)-getattr(wanted.pose.pose.position, axis)) < 1e-6
        sensor = next(t for t in reversed(sensor_transforms)
                      if t.header.stamp == cloud.header.stamp)
        assert sensor.header.frame_id == "odom"
        yaw, origin = sensor_pose(end)
        expected_origin = transform(transform(extrinsic_xyz, yaw, origin),
                                    alignment_yaw, alignment_xyz)
        for i, axis in enumerate("xyz"):
            assert abs(getattr(sensor.transform.translation, axis) - expected_origin[i]) < 1e-6

    with tempfile.TemporaryDirectory(prefix="odin_raw_test_") as directory:
        path = Path(directory)/"node.log"
        with path.open("w") as log:
            proc = subprocess.Popen([sys.argv[1], "--ros-args", "-p", "self_filter_radius:=0.15",
                                     "-p", "max_stamp_age:=0.4", "-p", "near_outlier_range:=1.0",
                                     "-p", f"max_range:={max_range}"], stdout=log, stderr=log)
            try:
                until = time.monotonic()+5
                while any(p.get_subscription_count() == 0 for p in publishers.values()):
                    assert time.monotonic() < until, "Input discovery timed out"
                    spin(0.02)
                start, expected, _ = batch()
                calibration(False)
                spin(0.04)
                assert not outputs
                calibration()
                expect_cloud(0, expected, start)
                spin(0.12)
                before = len(outputs)
                start, expected, tail = batch(tail=False)
                spin(0.03)
                assert len(outputs) == before
                for msg in tail:
                    publishers["/odin1/odometry_highfreq"].publish(msg)
                expect_cloud(before, expected, start)
                # The device refines T_IL online; new exposure groups use the new calibration.
                extrinsic_xyz = (-0.026, 0.022, 0.036)
                spin(0.12)
                before = len(outputs)
                start, expected, _ = batch()
                expect_cloud(before, expected, start)
                for bad in ("offset", "layout", "quality"):
                    spin(0.12)
                    before = len(outputs)
                    batch(bad=bad)
                    spin(0.05)
                    assert len(outputs) == before, bad
                spin(0.12)
                batch(gap=True)
                spin(0.3)
                assert len(outputs) == before, "Missing exposure poses were accepted"
                start, expected, _ = batch()
                expect_cloud(before, expected, start)
                spin(0.45)
                before = len(outputs)
                start, expected, _ = batch(with_reference=False)
                spin(0.04)
                assert len(outputs) == before, "Expired SLAM reference was accepted"
                reference(start+25_000_000)
                expect_cloud(before, expected, start)
                spin(0.12)
                reference(time.time_ns()-80_000_000, shift=0.1)
                spin(0.03)
                before = len(outputs)
                batch()
                spin(0.08)
                assert len(outputs) == before, "Changed alignment was accepted"
                assert proc.poll() is None
            finally:
                proc.send_signal(signal.SIGINT)
                proc.wait(timeout=5)
                print(path.read_text())
    node.destroy_node()
    rclpy.shutdown()
    print("PASS: 32-group deskew, nonzero T_IL/alignment, XYZI, confidence/body/range filters, pose coverage, freshness and alignment changes")


if __name__ == "__main__":
    main()
