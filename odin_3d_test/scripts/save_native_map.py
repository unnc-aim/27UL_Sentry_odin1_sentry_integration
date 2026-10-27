#!/usr/bin/env python3
"""Confirm transfer completion through ROS messages or the local driver log."""
import argparse
import math
import time
from pathlib import Path
import rclpy
from rclpy.logging import get_logging_directory
from rcl_interfaces.msg import Log
from odin_ros_driver.srv import SaveMap


class DriverEvents:
    """Read newly appended records belonging to this output's save request."""

    def __init__(self, directory, output):
        self.directory = directory
        self.trigger = f"Map save triggered, will transfer to [{output}]"
        self.files = {
            p: (p.stat().st_size, b"", False)
            for p in directory.glob("host_sdk_sample_*.log")
        }

    def poll(self):
        events = []
        for path in self.directory.glob("host_sdk_sample_*.log"):
            offset, pending, active = self.files.get(path, (0, b"", False))
            try:
                with path.open("rb") as stream:
                    if path.stat().st_size < offset:
                        offset, pending, active = 0, b"", False
                    stream.seek(offset)
                    data = pending + stream.read(1024 * 1024)
                    offset = stream.tell()
            except OSError:
                continue
            lines = data.split(b"\n")
            for raw in lines[:-1]:
                line = raw.decode(errors="replace")
                if self.trigger in line:
                    active = True
                if active and "Map transfer " in line:
                    events.append(line)
            self.files[path] = (offset, lines[-1], active)
        return events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=180.0)
    args, ros = parser.parse_known_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("Positive timeout required")
    path = args.output.expanduser().resolve()
    if path.exists():
        parser.error("Choose a new map file in the driver configuration")
    rclpy.init(args=ros)
    node = rclpy.create_node("save_odin_native_map")
    events = []
    node.create_subscription(
        Log,
        "/rosout",
        lambda m: events.append(m.msg) if "Map transfer " in m.msg else None,
        100,
    )
    client = node.create_client(SaveMap, "/odin1/save_map")
    end = time.monotonic() + args.timeout
    try:
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError("SaveMap service unavailable")
        driver_events = DriverEvents(get_logging_directory(), path)
        future = client.call_async(SaveMap.Request(value=1))
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            events.extend(driver_events.poll())
            if future.done():
                result = future.result()
                if not result.success:
                    raise RuntimeError(f"Save request failed: {result.rc}")
            if any(
                "Map transfer failed" in e or "Map transfer skipped" in e
                for e in events
            ):
                raise RuntimeError(str(events))
            if future.done() and any(
                e.strip().endswith("Map transfer completed: " + str(path))
                for e in events
            ):
                if not path.is_file() or path.stat().st_size == 0:
                    raise RuntimeError("Completed map file missing")
                print("Transfer completed:", path)
                return
        raise RuntimeError(
            "Transfer completion not confirmed; preserve the driver and inspect its log"
        )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
