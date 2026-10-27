import os

os.environ["ROS_DOMAIN_ID"] = "116"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import sys, struct, subprocess, time, tempfile
from pathlib import Path
import numpy as np, yaml, rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from capture_color_map import decode

rclpy.init()
node = rclpy.create_node("test_color_filter")
received = []
sub = node.create_subscription(
    PointCloud2, "/odin_3d/color_filtered", received.append, 10
)
op = node.create_publisher(Odometry, "/odin1/odometry", 10)
cp = node.create_publisher(PointCloud2, "/odin1/cloud_slam", 10)
params = yaml.safe_load(
    (
        Path(get_package_share_directory("odin_sentry_bringup"))
        / "config/self_filter.yaml"
    ).read_text()
)["odin_nav_adapter"]["ros__parameters"]
# Exercise both cloud orientations without tripping the adapter pose-step check.
params.update(max_translation_step=5.0, max_rotation_step=2.0)
with tempfile.TemporaryDirectory() as tmp:
    file = Path(tmp) / "filter.yaml"
    file.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
    proc = subprocess.Popen(
        [
            get_package_prefix("odin_3d_test") + "/lib/odin_3d_test/color_filter",
            "--ros-args",
            "--params-file",
            str(file),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    adapter = subprocess.Popen(
        [
            get_package_prefix("odin_sentry_bringup")
            + "/lib/odin_sentry_bringup/odin_nav_adapter",
            "--ros-args",
            "--params-file",
            str(file),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    try:
        for rotated in (False, True):
            received.clear()
            end = time.monotonic() + 1.8
            expected = {0x112200 + i for i in range(8)} | {0xFFAA00}
            points = [(0.1, 0, 0, 1), (0.3, 0.2, -0.3, 2), (0.6, 0.6, 0.1, 3)]
            points += [(0.7 + i * 0.002, -0.2, -0.1, 0x112200 + i) for i in range(8)]
            points += [(3.0, 0, 0, 0xFFAA00), (float("nan"), 0, 0, 4)]
            while time.monotonic() < end:
                msg = Odometry()
                msg.header.frame_id = "odom"
                msg.child_frame_id = "imu"
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.pose.pose.orientation.w = 1.0
                data = points
                if rotated:
                    msg.pose.pose.position.x = 3.0
                    msg.pose.pose.position.y = 2.0
                    msg.pose.pose.position.z = 1.0
                    msg.pose.pose.orientation.z = 2 ** -0.5
                    msg.pose.pose.orientation.w = 2 ** -0.5
                    data = [(3 - y, 2 + x, 1 + z, c) for x, y, z, c in points]
                cloud = PointCloud2()
                cloud.header = msg.header
                cloud.height = 1
                cloud.width = len(data)
                cloud.point_step = 16
                cloud.row_step = 16 * len(data)
                cloud.fields = [
                    PointField(
                        name=k, offset=i * 4, datatype=PointField.FLOAT32, count=1
                    )
                    for i, k in enumerate(["x", "y", "z", "rgb"])
                ]
                cloud.data = b"".join(struct.pack("<fffI", *p) for p in data)
                op.publish(msg)
                cp.publish(cloud)
                for _ in range(8):
                    rclpy.spin_once(node, timeout_sec=0.01)
            assert received, ("No synchronized filtered cloud", rotated)
            xyz, rgb = decode(received[-1])
            assert set(rgb) == expected, (rotated, rgb)
            assert received[-1].header.frame_id == "odin_local"
        print(
            "Existing body sphere, rotating body box, near outlier filter and RGB retention passed"
        )
    except Exception:
        for child in (proc, adapter):
            child.terminate()
            child.wait(timeout=5)
            print(child.stderr.read().decode())
        raise
    finally:
        proc.terminate()
        proc.wait(timeout=5)
        adapter.terminate()
        adapter.wait(timeout=5)
node.destroy_node()
rclpy.shutdown()
