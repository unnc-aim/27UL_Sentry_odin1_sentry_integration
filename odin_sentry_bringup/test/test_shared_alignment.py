"""Keep device-world TF identical to the alignment used by the navigation pose."""
import os

os.environ["ROS_DOMAIN_ID"] = "119"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import sys, time, subprocess
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from std_msgs.msg import Header

rclpy.init()
n = rclpy.create_node("shared_alignment_test")
proc = subprocess.Popen(
    [sys.argv[1]], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE
)
pub = n.create_publisher(Odometry, "/odin1/odometry", 5)
cloud_pub = n.create_publisher(PointCloud2, "/odin1/cloud_slam", 5)
aligned_pub = n.create_publisher(Odometry, "/lidar_odometry", 5)
static = StaticTransformBroadcaster(n)
mount = TransformStamped()
mount.header.frame_id = "base_footprint"
mount.child_frame_id = "odin_imu"
mount.transform.translation.z = 0.43
mount.transform.rotation.w = 1.0
static.sendTransform(mount)
anchors = []
sub = n.create_subscription(
    TFMessage,
    "/tf_static",
    lambda m: anchors.extend(
        t for t in m.transforms if t.child_frame_id == "odin_local"
    ),
    QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL),
)
A = np.eye(4)
A[:3, :3] = Rotation.from_euler("z", 0.4).as_matrix()
A[:3, 3] = [-0.2, 0.3, 0.43]


def fill(m, T):
    p = m.pose.pose.position
    p.x, p.y, p.z = map(float, T[:3, 3])
    q = m.pose.pose.orientation
    q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(T[:3, :3]).as_quat())


def run(seconds, aligned=False, mismatch=False, yaw=0.2):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        assert proc.poll() is None
        t = n.get_clock().now()
        d = Odometry()
        d.header.stamp = t.to_msg()
        d.header.frame_id = "odom"
        d.child_frame_id = "imu"
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
        T[:3, 3] = [1.0, 2.0, 0.2]
        fill(d, T)
        pub.publish(d)
        fields = [
            PointField(name=k, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, k in enumerate(["x", "y", "z"])
        ]
        cloud_pub.publish(
            point_cloud2.create_cloud(
                Header(stamp=d.header.stamp, frame_id="odom"), fields, [(2.0, 2.0, 0.0)]
            )
        )
        if aligned:
            a = Odometry()
            a.header.frame_id = "odom"
            a.child_frame_id = "odin_imu"
            a.header.stamp = (
                (t - rclpy.duration.Duration(seconds=0.01)).to_msg()
                if mismatch
                else d.header.stamp
            )
            fill(a, A @ T)
            aligned_pub.publish(a)
        until = time.monotonic() + 0.05
        while time.monotonic() < until:
            rclpy.spin_once(n, timeout_sec=0.005)


try:
    run(1.2)
    assert (
        not anchors
    ), "Anchor selected an independent startup mount before aligned odometry"
    run(0.4, aligned=True, mismatch=True)
    assert not anchors, "Different measurement times paired for world alignment"
    run(0.7, aligned=True)
    assert anchors, "Missing shared alignment TF"
    run(0.4, aligned=True, yaw=0.7)
    for t in anchors:
        q = t.transform.rotation
        p = t.transform.translation
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3, 3] = [p.x, p.y, p.z]
        assert np.allclose(T, A, atol=1e-6), "TF differs from navigation alignment"
    print("Delayed aligned pose, exact timestamps and shared world transform passed")
finally:
    proc.terminate()
    proc.wait(timeout=5)
    n.destroy_node()
    rclpy.shutdown()
