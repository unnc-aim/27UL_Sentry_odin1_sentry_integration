"""Fresh native poses continue through delayed clouds; faults still stop output."""
import os
import math
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "142"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def main():
    rclpy.init()
    node = rclpy.create_node("independent_odometry_check")
    poses = node.create_publisher(Odometry, "/odin1/odometry", 5)
    clouds = node.create_publisher(PointCloud2, "/odin1/cloud_slam", 5)
    received, filtered = [], []
    subscriptions = [
        node.create_subscription(
            Odometry, "/odin_navigation/odometry", received.append, 50),
        node.create_subscription(
            PointCloud2, "/odin_navigation/cloud", filtered.append, 10),
    ]
    fields = [PointField(name=k, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, k in enumerate(("x", "y", "z"))]

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.002)

    def publish(age=0., x=2., cloud=False, yaw=0.):
        msg = Odometry()
        msg.header = Header(
            stamp=(node.get_clock().now() -
                   rclpy.duration.Duration(seconds=age)).to_msg(),
            frame_id="odom")
        msg.child_frame_id = "imu"
        msg.pose.pose.position.x = x
        msg.pose.pose.orientation.z = math.sin(yaw/2)
        msg.pose.pose.orientation.w = math.cos(yaw/2)
        poses.publish(msg)
        if cloud:
            clouds.publish(point_cloud2.create_cloud(
                msg.header, fields, [(x, 0., 0.), (x + 1., 0., 0.)]))
        return msg

    def delivered(msg):
        return any(m.header.stamp == msg.header.stamp for m in received)

    with tempfile.TemporaryDirectory(prefix="odin_independent_") as directory:
        with (Path(directory) / "adapter.log").open("w+") as log:
            process = subprocess.Popen(
                [sys.argv[1], "--ros-args", "-p", "independent_odometry:=true",
                 "-p", "self_filter_radius:=0.15"], stdout=log, stderr=subprocess.STDOUT)
            try:
                end = time.monotonic() + 4
                while poses.get_subscription_count() == 0 or clouds.get_subscription_count() == 0:
                    assert time.monotonic() < end and process.poll() is None
                    spin(.02)
                spin(.2)
                publish(age=5.)
                spin(.1)
                assert not received, "Old pose reached navigation"
                # No SLAM clouds at all: native/raw navigation still needs timely poses.
                samples = []
                for _ in range(20):
                    msg = publish()
                    spin(.06)
                    samples.append(delivered(msg))
                assert sum(
                    samples) >= 18, f"Pose delivery waited for clouds: {sum(samples)}/20"
                assert all(m.header.frame_id == "odin_local" and m.child_frame_id == "odin_imu"
                           and m.pose.pose.position.x == 2. for m in received)
                # A delayed cloud retains its own paired pose and measurement time.
                old = publish()
                spin(.04)
                latest = publish(x=2.4)
                spin(.04)
                assert delivered(latest), "New pose waited for an older cloud"
                clouds.publish(point_cloud2.create_cloud(
                    old.header, fields, [(2., 0., 0.), (3., 0., 0.)]))
                spin(.12)
                assert any(m.header.stamp == old.header.stamp and m.width == 1 for m in filtered), \
                    "Cloud filtering lost its measurement-time pose"
                before = len(received)
                publish(age=1., x=2.4)
                spin(.1)
                assert len(
                    received) == before, "Expired pose was retimed or forwarded"
                # A short pose outage still requires three new samples to recover.
                spin(.65)
                for index in range(3):
                    msg = publish(x=2.4)
                    spin(.07)
                    assert delivered(msg) == (
                        index == 2), "Pose recovery count changed"
                # Fresh poses continue after more than two seconds with no cloud.
                for _ in range(35):
                    msg = publish(x=2.4)
                    spin(.06)
                assert delivered(msg), "Cloud absence stopped fresh odometry"
                for _ in range(5):
                    msg = publish(x=2.4, cloud=True)
                    spin(.06)
                assert delivered(msg) and any(
                    m.header.stamp == msg.header.stamp for m in filtered
                ), "Returning clouds stopped continuous odometry"
                # Long outages keep output stopped and recover with three valid poses.
                before = len(received)
                spin(2.15)
                assert len(received) == before
                for index in range(3):
                    msg = publish(x=2.4)
                    spin(.07)
                    assert delivered(msg) == (index == 2), "Long pose outage stayed latched"
                # Buffered measurements still describe a smooth turn, even when
                # their age excludes them from navigation output.
                before = len(received)
                spin(.45)
                for index in range(6):
                    publish(age=.35, x=2.4, yaw=.3*(index+1))
                    spin(.1)
                assert len(received) == before, "Delayed turn samples reached navigation"
                for index in range(3):
                    msg = publish(x=2.4, yaw=1.8)
                    spin(.07)
                    assert delivered(msg) == (index == 2), "Continuous buffered turn became a pose fault"
                for yaw in [1.2, .6, 0.]:
                    publish(x=2.4, yaw=yaw)
                    spin(.06)
                before = len(received)
                publish(x=5.)
                spin(.1)
                for _ in range(4):
                    publish(x=2.4)
                    spin(.05)
                assert len(
                    received) == before, "Pose jump resumed the old frame"
                print(
                    "PASS: independent pose timing, delayed cloud geometry, age, recovery and jump handling")
            except BaseException:
                log.flush()
                log.seek(0)
                print(log.read()[-2500:])
                raise
            finally:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
