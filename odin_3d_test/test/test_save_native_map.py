"""Exercise a log-only SaveMap completion in an isolated ROS domain."""
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "119"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

import rclpy
from odin_ros_driver.srv import SaveMap
from save_native_map import DriverEvents


with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    output = root / "native.bin"
    log = root / "host_sdk_sample_test.log"
    trigger = f"Map save triggered, will transfer to [{output}]\n"
    complete = f"Map transfer completed: {output}\n"
    log.write_text(trigger + complete)
    reader = DriverEvents(root, output)
    assert reader.poll() == [], "Historical completion belongs to an earlier request"
    with log.open("a") as stream:
        stream.write("Map transfer failed, error code: 1\n" + trigger + complete[:20])
    assert reader.poll() == [], "Wait for this request's complete log line"
    with log.open("a") as stream:
        stream.write(complete[20:])
    assert reader.poll() == [complete.strip()]
    assert reader.poll() == []

    rclpy.init()
    node = rclpy.create_node("save_map_test_service", enable_rosout=False)
    calls = []

    def save(request, response):
        calls.append(request.value)
        output.write_bytes(b"synthetic saved map")
        with log.open("a") as stream:
            stream.write(trigger + complete)
        response.success = True
        response.rc = 0
        return response

    service = node.create_service(SaveMap, "/odin1/save_map", save)
    child = subprocess.Popen(
        [sys.executable, str(Path(__file__).parents[1] / "scripts/save_native_map.py"),
         "--output", str(output), "--timeout", "10"],
        env={**os.environ, "ROS_LOG_DIR": str(root)},
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    try:
        end = time.monotonic() + 12
        while child.poll() is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert child.poll() is not None, "Save helper waited after driver completion"
        text = child.communicate(timeout=2)[0]
        assert child.returncode == 0, text
        assert calls == [1]
        assert "Transfer completed:" in text, text
    finally:
        if child.poll() is None:
            child.terminate()
            child.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()

print("Log-only completion and old/partial event handling passed")
