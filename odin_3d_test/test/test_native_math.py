import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
import numpy as np
from native_math import Alignment, close, fresh
from scipy.spatial.transform import Rotation

anchor = np.eye(4)
anchor[:3, 3] = [0.13, 0.07, 0.43]
device = np.eye(4)
device[:3, :3] = Rotation.from_euler("z", 0.7).as_matrix()
device[:3, 3] = [3, 2, 1]
map_odom = np.linalg.inv(device) @ np.linalg.inv(anchor)
assert np.allclose(map_odom @ anchor @ device, np.eye(4))
a = Alignment()
for _ in range(3):
    a.observe(map_odom)
assert a.pose is not None and not a.fault
bad = map_odom.copy()
bad[0, 3] += 1
a.observe(bad)
assert a.fault
assert fresh(9.9, 10, 0.5) and not fresh(9, 10, 0.5) and not fresh(11, 10, 0.5)
print("Transform composition, confirmation, jump and timestamp checks passed")
