"""A current surface remains usable between input frames and expires on measurement age."""
import os

os.environ["ROS_DOMAIN_ID"] = "115"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import subprocess, time, struct
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from rclpy.duration import Duration
from ament_index_python.packages import get_package_prefix

rclpy.init()
node = rclpy.create_node("terrain_freshness_test")
proc = subprocess.Popen(
    [get_package_prefix("odin_3d_test") + "/lib/odin_3d_test/terrain_surface"],
    stdout=subprocess.DEVNULL,
    stderr=subprocess.PIPE,
)
pub = node.create_publisher(PointCloud2, "/odin_obstacles/registered_scan", 1)
values = []
surfaces = []
surface_sub = node.create_subscription(
    OccupancyGrid, "/odin_3d/surface", surfaces.append, 10
)
sub = node.create_subscription(
    Bool, "/odin_3d/terrain_ready", lambda m: values.append(m.data), 10
)
br = StaticTransformBroadcaster(node)
tf = TransformStamped()
tf.header.frame_id = "odom"
tf.child_frame_id = "base_footprint"
tf.transform.rotation.w = 1.0
br.sendTransform(tf)


def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


cloud = PointCloud2()
cloud.header.frame_id = "odom"
cloud.height = 1
cloud.point_step = 12
cloud.fields = [
    PointField(name=k, offset=i * 4, datatype=PointField.FLOAT32, count=1)
    for i, k in enumerate(["x", "y", "z"])
]
cloud.data = b"".join(
    struct.pack("<fff", x * 0.02, y * 0.02, 0.0)
    for x in range(-50, 51)
    for y in range(-50, 51)
)
cloud.width = len(cloud.data) // 12
cloud.row_step = len(cloud.data)
try:
    spin(1.0)
    # Send one frame and await the first valid surface.
    cloud.header.stamp = node.get_clock().now().to_msg()
    pub.publish(cloud)
    end = time.monotonic() + 0.45
    while time.monotonic() < end and not any(values):
        rclpy.spin_once(node, timeout_sec=0.01)
    assert any(values), "No valid plane received"
    assert surfaces and abs(surfaces[-1].info.resolution - .05) < 1e-6
    assert surfaces[-1].info.width == surfaces[-1].info.height == 120
    values.clear()
    spin(0.21)
    assert values and all(values), "Fresh surface lost between input frames"
    values.clear()
    spin(0.6)
    assert values and not values[-1], "Old measurement should expire"
    # A current view containing only a ceiling retains unknown terrain cells;
    # observation health stays valid so the costmap can use the surveyed map.
    floor_data = cloud.data
    cloud.data = b"".join(
        struct.pack("<fff", x * 0.02, y * 0.02, 1.8)
        for x in range(-50, 51)
        for y in range(-50, 51)
    )
    values.clear()
    surfaces.clear()
    cloud.header.stamp = node.get_clock().now().to_msg()
    pub.publish(cloud)
    spin(0.3)
    assert surfaces and all(v == -1 for v in surfaces[-1].data)
    assert values and values[-1], "A fresh unknown grid was mistaken for missing data"
    spin(0.6)
    assert not values[-1], "Unknown terrain observation must still expire"
    cloud.data = floor_data
    # Use another base publisher with delayed measurement stamps after the first process exits.
    proc.terminate()
    proc.wait(timeout=5)
    proc = subprocess.Popen(
        [
            get_package_prefix("odin_3d_test") + "/lib/odin_3d_test/terrain_surface",
            "--ros-args",
            "-r",
            "/tf:=/delayed_tf",
            "-r",
            "/tf_static:=/delayed_tf_static",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    delayed = node.create_publisher(
        __import__("tf2_msgs.msg", fromlist=["TFMessage"]).TFMessage, "/delayed_tf", 10
    )
    from tf2_msgs.msg import TFMessage

    end = time.monotonic() + 2.5
    values.clear()
    while time.monotonic() < end:
        now = node.get_clock().now()
        tf.header.stamp = (now - Duration(seconds=0.15)).to_msg()
        delayed.publish(TFMessage(transforms=[tf]))
        cloud.header.stamp = now.to_msg()
        pub.publish(cloud)
        spin(0.05)
    assert values[-5:] and all(
        values[-5:]
    ), "Select fresh earlier cloud while the latest cloud awaits TF"
    print("Inter-frame validity, measurement expiry and delayed TF selection passed")
finally:
    proc.terminate()
    proc.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()
