"""Record and export filtered geometry in an isolated ROS domain."""
import os

os.environ["ROS_DOMAIN_ID"] = "117"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import json, subprocess, tempfile, time, struct
from pathlib import Path
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2, PointField
from ament_index_python.packages import get_package_prefix

rclpy.init()
node = rclpy.create_node("recording_input_test")
cloud_pub = node.create_publisher(PointCloud2, "/odin_obstacles/registered_scan", 10)
tf_pub = node.create_publisher(TFMessage, "/tf", 10)
exe = Path(get_package_prefix("odin_3d_test")) / "lib/odin_3d_test"
try:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        native = root / "native.bin"
        native.write_bytes(b"test fixture")
        output = root / "observations.ply"
        child = subprocess.Popen(
            [
                str(exe / "record_navigation_cloud.py"),
                "--native-map",
                str(native),
                "--output",
                str(output),
                "--seconds",
                "4",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        cloud = PointCloud2()
        cloud.header.frame_id = "odom"
        cloud.height = 1
        cloud.point_step = 16
        cloud.fields = [
            PointField(name=k, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, k in enumerate(["x", "y", "z", "intensity"])
        ]
        cloud.data = b"".join(
            struct.pack("<ffff", x * 0.02, y * 0.02, 0.0, 0.0)
            for x in range(-50, 51)
            for y in range(-50, 51)
        )
        cloud.width = len(cloud.data) // 16
        cloud.row_step = len(cloud.data)
        end = time.monotonic() + 6
        while child.poll() is None and time.monotonic() < end:
            now = node.get_clock().now()
            first = TransformStamped()
            first.header.frame_id = "map"
            first.child_frame_id = "odom"
            first.header.stamp = now.to_msg()
            first.transform.rotation.w = 1.0
            first.transform.translation.x = 1.0
            second = TransformStamped()
            second.header.frame_id = "odom"
            second.child_frame_id = "base_footprint"
            second.header.stamp = now.to_msg()
            second.transform.rotation.w = 1.0
            tf_pub.publish(TFMessage(transforms=[first, second]))
            cloud.header.stamp = (now - Duration(seconds=0.05)).to_msg()
            cloud_pub.publish(cloud)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.02)
        if child.poll() is None:
            child.terminate()
        text = child.communicate(timeout=5)[0]
        assert child.returncode == 0, text
        meta = json.loads(output.with_suffix(".json").read_text())
        assert (
            meta["frame"] == "map"
            and meta["source_frames"] > 0
            and meta["seed_ground_xyz"] == [1.0, 0.0, 0.0]
        )
        result = subprocess.run(
            [
                str(exe / "build_navigation_map.py"),
                "--recording",
                str(output),
                "--output",
                str(root / "nav"),
            ],
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stderr + result.stdout
        assert (root / "nav/navigation.yaml").is_file()
        meta["capacity_reached"] = True
        output.with_suffix(".json").write_text(json.dumps(meta))
        result = subprocess.run(
            [
                str(exe / "build_navigation_map.py"),
                "--recording",
                str(output),
                "--output",
                str(root / "incomplete"),
            ],
            capture_output=True,
            text=True,
        )
        assert result.returncode != 0 and not (root / "incomplete").exists()
        print(
            "Filtered raw recording, native-frame conversion, ground seed and checked map export passed"
        )
finally:
    node.destroy_node()
    rclpy.shutdown()
