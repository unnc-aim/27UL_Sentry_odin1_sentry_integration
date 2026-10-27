#!/usr/bin/env python3
"""Save a bounded live RGB point-cloud preview; optimized geometry is exported from BIN separately."""
import argparse, json, time
from pathlib import Path
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import qos_profile_sensor_data


def decode(msg, require_color=True):
    fields = {f.name: f for f in msg.fields}
    color = "rgb" if "rgb" in fields else "rgba" if "rgba" in fields else None
    names = ("x", "y", "z") + ((color,) if color else ())
    if require_color and color is None:
        raise ValueError("XYZ and RGB fields required")
    if not all(k in fields for k in names):
        raise ValueError("XYZ and RGB fields required")
    if any(
        fields[k].datatype != PointField.FLOAT32 or fields[k].count != 1
        for k in ("x", "y", "z")
    ):
        raise ValueError("FLOAT32 XYZ required")
    if color and fields[color].datatype not in (PointField.FLOAT32, PointField.UINT32):
        raise ValueError("Packed RGB required")
    order = ">" if msg.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"] + (["rgb"] if color else []),
            "formats": [order + "f4"] * 3 + ([order + "u4"] if color else []),
            "offsets": [fields[k].offset for k in names],
            "itemsize": msg.point_step,
        }
    )
    a = np.ndarray(
        (msg.height, msg.width),
        dtype=dtype,
        buffer=msg.data,
        strides=(msg.row_step, msg.point_step),
    ).reshape(-1)
    xyz = np.column_stack([a[k] for k in ("x", "y", "z")])
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid], a["rgb"][valid] if color else np.zeros(
        np.count_nonzero(valid), np.uint32
    )


def save(path, xyz, rgb):
    values = np.empty(
        len(xyz),
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
        ],
    )
    for i, k in enumerate(("x", "y", "z")):
        values[k] = xyz[:, i]
    for shift, k in [(16, "red"), (8, "green"), (0, "blue")]:
        values[k] = (rgb >> shift) & 255
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as f:
        f.write(
            (
                "ply\nformat binary_little_endian 1.0\nelement vertex "
                + str(len(values))
                + "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
            ).encode()
        )
        f.write(values.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--voxel", type=float, default=0.08)
    parser.add_argument("--max-points", type=int, default=300000)
    args, ros = parser.parse_known_args()
    if (
        args.output.exists()
        or args.seconds <= 0
        or args.voxel <= 0
        or args.max_points < 1
    ):
        parser.error("New file and positive limits required")
    rclpy.init(args=ros)
    node = rclpy.create_node("color_map_capture")
    xyz = np.empty((0, 3), np.float32)
    rgb = np.empty(0, np.uint32)
    last = 0.0

    def callback(msg):
        nonlocal xyz, rgb, last
        if time.monotonic() - last < 0.5 or msg.header.frame_id != "odin_local":
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not -0.05 <= node.get_clock().now().nanoseconds * 1e-9 - stamp <= 0.5:
            return
        last = time.monotonic()
        points, colors = decode(msg)
        xyz = np.concatenate([xyz, points])
        rgb = np.concatenate([rgb, colors])
        _, idx = np.unique(
            np.floor(xyz / args.voxel).astype(np.int64), axis=0, return_index=True
        )
        if len(idx) > args.max_points:
            idx = idx[np.linspace(0, len(idx) - 1, args.max_points, dtype=int)]
        xyz = xyz[idx]
        rgb = rgb[idx]

    node.create_subscription(
        PointCloud2, "/odin_3d/color_filtered", callback, qos_profile_sensor_data
    )
    end = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if not len(xyz):
        raise RuntimeError("No fresh colored cloud received")
    save(args.output, xyz, rgb)
    args.output.with_suffix(".json").write_text(
        json.dumps(
            {
                "frame": "odin_local",
                "kind": "live_preview",
                "optimized_geometry": False,
                "points": len(xyz),
                "voxel_m": args.voxel,
            },
            indent=2,
        )
    )
    print("Saved RGB preview:", args.output)


if __name__ == "__main__":
    main()
