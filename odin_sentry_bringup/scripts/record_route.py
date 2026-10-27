#!/usr/bin/env python3
"""Record ordered map poses selected with RViz's standard 2D Goal Pose tool."""
import math
import os
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


def pose_values(pose, grid):
    """Validate a planar pose and its destination cell in the selected map."""
    if pose.header.frame_id != grid.header.frame_id:
        raise ValueError("请选择 map 坐标中的点")
    p, q = pose.pose.position, pose.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("点位坐标需要为有限数值")
    if abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1.0) > 0.01:
        raise ValueError("点位朝向需要有效的单位四元数")
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    origin = grid.info.origin
    oq = origin.orientation
    angle = math.atan2(2 * (oq.w * oq.z + oq.x * oq.y),
                       1 - 2 * (oq.y * oq.y + oq.z * oq.z))
    dx, dy = p.x - origin.position.x, p.y - origin.position.y
    x = math.floor((math.cos(angle) * dx + math.sin(angle) * dy)
                   / grid.info.resolution)
    y = math.floor((-math.sin(angle) * dx + math.cos(angle) * dy)
                   / grid.info.resolution)
    if not (0 <= x < grid.info.width and 0 <= y < grid.info.height):
        raise ValueError("请选择地图范围内的点")
    if grid.data[y * grid.info.width + x] != 0:
        raise ValueError("请选择白色空闲区域，并为车体留出空间")
    return [float(p.x), float(p.y), yaw]


def save_route(path, map_file, points):
    """Store map location relative to the route file for workspace portability."""
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "map": os.path.relpath(map_file, path.parent),
        "frame_id": "map",
        "waypoints": [
            dict(x=round(x, 4), y=round(y, 4), yaw=round(yaw, 4))
            for x, y, yaw in points
        ],
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


class RouteRecorder(Node):
    def __init__(self):
        super().__init__("odin_route_recorder")
        self.path = Path(self.declare_parameter("output_file", "").value).resolve()
        self.map_file = Path(self.declare_parameter("map_file", "").value).resolve()
        if not self.map_file.is_file():
            raise ValueError("map_file 需要指向地图 YAML")
        if self.path.exists():
            raise ValueError(f"请为新点列选择新的保存文件：{self.path}")
        self.grid = None
        self.points = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, "/odin_route/markers", qos)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.create_subscription(PoseStamped, "/odin_route/goal_pose", self.on_pose, 10)
        self.create_service(Trigger, "/odin_route/undo", self.undo)
        self.get_logger().info(f"点位按点击顺序自动保存：{self.path}")

    def on_map(self, grid):
        if grid.info.resolution > 0 and len(grid.data) == grid.info.width * grid.info.height:
            self.grid = grid

    def on_pose(self, pose):
        if self.grid is None:
            self.get_logger().warning("等待地图加载")
            return
        try:
            point = pose_values(pose, self.grid)
            save_route(self.path, self.map_file, self.points + [point])
        except (ValueError, OSError) as exc:
            self.get_logger().error(str(exc))
            return
        self.points.append(point)
        self.draw()
        self.get_logger().info(
            f"已保存第 {len(self.points)} 点：x={point[0]:.3f}, "
            f"y={point[1]:.3f}, yaw={point[2]:.3f}"
        )

    def undo(self, request, response):
        if not self.points:
            response.message = "当前点列为空"
            return response
        try:
            save_route(self.path, self.map_file, self.points[:-1])
        except OSError as exc:
            response.message = str(exc)
            return response
        self.points.pop()
        self.draw()
        response.success = True
        response.message = f"已撤销末点，当前共 {len(self.points)} 点"
        return response

    def draw(self):
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        stamp = self.get_clock().now().to_msg()
        line = Marker()
        line.header.frame_id, line.header.stamp = "map", stamp
        line.ns, line.id, line.type = "route", 0, Marker.LINE_STRIP
        line.pose.orientation.w = 1.0
        line.scale.x = 0.04
        line.color.r, line.color.g, line.color.b, line.color.a = 0.0, 0.65, 1.0, 1.0
        for index, (x, y, yaw) in enumerate(self.points, 1):
            line.points.append(Point(x=x, y=y, z=0.04))
            arrow = Marker()
            arrow.header = line.header
            arrow.ns, arrow.id, arrow.type = "poses", index, Marker.ARROW
            arrow.pose.position = Point(x=x, y=y, z=0.07)
            arrow.pose.orientation.z = math.sin(yaw / 2)
            arrow.pose.orientation.w = math.cos(yaw / 2)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.55, 0.12, 0.12
            arrow.color = line.color
            label = Marker()
            label.header = line.header
            label.ns, label.id, label.type = "labels", index, Marker.TEXT_VIEW_FACING
            label.pose.position = Point(x=x, y=y + 0.3, z=0.15)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.4
            label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 0.3, 0.0, 1.0
            label.text = str(index)
            markers.markers.extend([arrow, label])
        if len(self.points) > 1:
            markers.markers.append(line)
        self.publisher.publish(markers)


def main():
    rclpy.init()
    node = RouteRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
