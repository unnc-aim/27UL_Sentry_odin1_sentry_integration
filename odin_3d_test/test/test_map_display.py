import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from color_map_display import ColorVoxels
from capture_color_map import decode
from builtin_interfaces.msg import Time

v = ColorVoxels(0.1, 3)
v.add(
    np.array([[1.0, 0.0, 0.0], [1.02, 0.0, 0.0]]),
    np.array([0xFF0000, 0xFF0000], dtype=np.uint32),
)
v.add(np.array([[0.0, 1.0, 0.0]]), np.array([0x00FF00], dtype=np.uint32))
points, colors = decode(v.cloud(Time()))
assert len(points) == 2 and set(colors) == {0xFF0000, 0x00FF00}
v.add(np.array([[2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]), np.array([0, 0], dtype=np.uint32))
assert len(v.cells) == 3 and v.full
assert (10, 0, 0) in v.cells and (0, 10, 0) in v.cells
print(
    "Past views retained, RGB preserved, duplicate voxels reduced, memory count bounded"
)
