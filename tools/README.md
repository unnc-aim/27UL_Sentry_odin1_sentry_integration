# Native map converter

`map_to_ply_amd64` is the existing x86-64 Odin map conversion executable copied from `/home/soyo/odin_lab/tools/map_to_ply_amd64` on 2026-10-27. The executable bytes are preserved. It converts a saved native BIN map into PLY for the existing surface exporter. The original executable remains in place.

Usage: `map_to_ply_amd64 INPUT.bin OUTPUT.ply`.

The workspace installs this tool as `ros2 run odin_3d_test map_to_ply_amd64`.
