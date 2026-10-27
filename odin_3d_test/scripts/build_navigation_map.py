#!/usr/bin/python3 -s
"""Export navigation geometry using the map-frame seed captured with filtered raw observations."""
import argparse
import json
import math
import subprocess
from pathlib import Path
from ament_index_python.packages import get_package_prefix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--obstacle-support", type=int, default=8)
    parser.add_argument("--ground-support", type=int, default=1)
    parser.add_argument("--ground-gap", type=float, default=0.10)
    parser.add_argument("--ground-mode", choices=["flat", "local"], default="flat")
    args = parser.parse_args()
    source = args.recording.expanduser().resolve()
    out = args.output.expanduser().resolve()
    meta = json.loads(source.with_suffix(".json").read_text())
    seed = meta.get("seed_ground_xyz")
    if (
        meta.get("frame") != "map"
        or meta.get("source_topic") != "/odin_obstacles/registered_scan"
        or meta.get("capacity_reached")
        or not isinstance(seed, list)
        or len(seed) != 3
        or not all(math.isfinite(v) for v in seed)
    ):
        parser.error(
            "Use a complete filtered-raw recording with a valid native map seed"
        )
    executable = (
        Path(get_package_prefix("odin_3d_test")) / "lib/odin_3d_test/export_surface"
    )
    subprocess.run(
        [
            str(executable),
            str(source),
            str(out),
            *[str(v) for v in seed],
            str(args.resolution),
            str(args.obstacle_support),
            args.ground_mode,
            str(args.ground_support),
            str(args.ground_gap),
        ],
        check=True,
    )
    meta.update(
        navigation_map="navigation.yaml",
        resolution_m=args.resolution,
        static_obstacle_support=args.obstacle_support,
        static_ground_support=args.ground_support,
        flat_ground_gap_m=args.ground_gap,
    )
    (out / "recording_metadata.json").write_text(json.dumps(meta, indent=2))
    print("Navigation map:", out / "navigation.yaml")


if __name__ == "__main__":
    main()
