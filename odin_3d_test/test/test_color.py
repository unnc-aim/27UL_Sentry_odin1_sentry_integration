import sys, struct, tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from capture_color_map import decode, save
from sensor_msgs.msg import PointCloud2, PointField

for endian in ["<", ">"]:
    msg = PointCloud2()
    msg.height = 1
    msg.width = 2
    msg.point_step = 16
    msg.row_step = 40
    msg.is_bigendian = endian == ">"
    msg.fields = [
        PointField(name=k, offset=i * 4, datatype=PointField.FLOAT32, count=1)
        for i, k in enumerate(["x", "y", "z", "rgb"])
    ]
    msg.data = struct.pack(
        endian + "fffIfffI", 1.0, 2.0, 3.0, 0xFF0000, 4.0, 5.0, 6.0, 0x00FF00
    ) + bytes(8)
    xyz, rgb = decode(msg)
    assert xyz.tolist() == [[1, 2, 3], [4, 5, 6]]
    assert rgb.tolist() == [0xFF0000, 0x00FF00]
    with tempfile.TemporaryDirectory() as root:
        p = Path(root) / "color.ply"
        save(p, xyz, rgb)
        data = p.read_bytes().split(b"end_header\n", 1)[1]
        assert data[12:15] == bytes([255, 0, 0]) and data[27:30] == bytes([0, 255, 0])
print("Packed RGB, row padding, byte order and PLY colors passed")

msg.fields = msg.fields[:3]
xyz, rgb = decode(msg, require_color=False)
assert xyz.tolist() == [[1, 2, 3], [4, 5, 6]] and rgb.tolist() == [0, 0]
try:
    decode(msg)
except ValueError:
    pass
else:
    raise AssertionError("Color capture still requires RGB")
