"""The reused terrain node preserves exposure timestamps down to nanoseconds."""
import os
os.environ["ROS_DOMAIN_ID"] = "118"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import struct
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import PointCloud2, PointField

rclpy.init()
node = rclpy.create_node("terrain_stamp_test")
publisher = node.create_publisher(PointCloud2, "registered_scan", 5)
received = []
subscription = node.create_subscription(
    PointCloud2, "terrain_map", lambda m: received.append(m.header.stamp), 5)
process = subprocess.Popen(
    [get_package_prefix("terrain_analysis") + "/lib/terrain_analysis/terrainAnalysis"],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    deadline = time.monotonic() + 4
    while not publisher.get_subscription_count() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
    assert publisher.get_subscription_count(), "Terrain subscriber missing"
    msg = PointCloud2()
    msg.header.frame_id = "odom"
    msg.height = 1
    msg.point_step = 16
    msg.fields = [PointField(name=k, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, k in enumerate(("x", "y", "z", "intensity"))]
    msg.data = b"".join(struct.pack("<ffff", x*.02, y*.02, 0., 0.)
                        for x in range(-20, 21) for y in range(-20, 21))
    msg.width = len(msg.data)//16
    msg.row_step = len(msg.data)
    for nanoseconds in (123456789, 234567891, 345678913):
        msg.header.stamp.sec = node.get_clock().now().to_msg().sec
        msg.header.stamp.nanosec = nanoseconds
        received.clear()
        publisher.publish(msg)
        deadline = time.monotonic()+1
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        assert received and received[-1] == msg.header.stamp, (received, msg.header.stamp)
    print("Terrain output preserves each input timestamp exactly")
finally:
    process.terminate()
    process.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()
