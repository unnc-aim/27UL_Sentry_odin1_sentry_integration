"""Exercise rotated map coordinates, ordered file saving, and undo."""
import math
from pathlib import Path
import runpy
import tempfile

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger


def main():
    module = runpy.run_path(str(Path(__file__).parents[1] / "scripts/record_route.py"))
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = 0.5
    grid.info.width, grid.info.height = 4, 4
    grid.info.origin.position.x, grid.info.origin.position.y = 10.0, -2.0
    grid.info.origin.orientation.z = math.sin(math.pi / 4)
    grid.info.origin.orientation.w = math.cos(math.pi / 4)
    grid.data = [0] * 16
    grid.data[0], grid.data[1] = -1, 100
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation.w = 1.0
    pose.pose.position.x, pose.pose.position.y = 9.25, -1.25
    assert module["pose_values"](pose, grid) == [9.25, -1.25, 0.0]
    for x, y in ((9.75, -1.75), (9.75, -1.25), (10.25, -2.25)):
        pose.pose.position.x, pose.pose.position.y = x, y
        try:
            module["pose_values"](pose, grid)
        except ValueError:
            pass
        else:
            raise AssertionError("Expected occupied, unknown or outside-map rejection")
    pose.pose.position.x, pose.pose.position.y = 9.25, -1.25
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        map_file = root / "map.yaml"
        map_file.write_text("image: map.pgm\n")
        output = root / "routes/test.yaml"
        rclpy.init(args=["--ros-args", "-p", f"map_file:={map_file}",
                        "-p", f"output_file:={output}"])
        node = module["RouteRecorder"]()
        try:
            node.on_map(grid)
            node.on_pose(pose)
            pose.pose.position.y = -0.75
            node.on_pose(pose)
            data = yaml.safe_load(output.read_text())
            assert (output.parent / data["map"]).resolve() == map_file
            assert data["waypoints"] == [
                {"x": 9.25, "y": -1.25, "yaw": 0.0},
                {"x": 9.25, "y": -0.75, "yaw": 0.0},
            ]
            response = node.undo(Trigger.Request(), Trigger.Response())
            assert response.success
            assert len(yaml.safe_load(output.read_text())["waypoints"]) == 1
            assert len(node.points) == 1
        finally:
            node.destroy_node()
            rclpy.shutdown()
    print("PASS: map coordinates, free-space selection, ordered route, relative map path, undo")


if __name__ == "__main__":
    main()
