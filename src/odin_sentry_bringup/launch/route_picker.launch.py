"""Open a saved map and record poses using RViz's 2D Goal Pose tool."""
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    map_file = Path(LaunchConfiguration("map").perform(context)).expanduser().resolve()
    output = Path(LaunchConfiguration("output_file").perform(context)).expanduser().resolve()
    if output.exists():
        raise ValueError(f"请为新点列选择新的保存文件：{output}")
    with map_file.open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    # Match the saved map bounds so the full image is visible on first launch.
    from PIL import Image
    image_file = Path(metadata["image"])
    if not image_file.is_absolute():
        image_file = map_file.parent / image_file
    with Image.open(image_file) as image:
        width, height = image.size
    share = Path(get_package_share_directory("odin_sentry_bringup"))
    config = yaml.safe_load((share / "config/route_picker.rviz").read_text())
    view = config["Visualization Manager"]["Views"]["Current"]
    resolution = float(metadata["resolution"])
    ox, oy, angle = metadata["origin"]
    import math
    dx, dy = width * resolution / 2, height * resolution / 2
    view.update(X=ox + math.cos(angle) * dx - math.sin(angle) * dy,
                Y=oy + math.sin(angle) * dx + math.cos(angle) * dy,
                Scale=min(850 / (width * resolution), 600 / (height * resolution)),
                Angle=float(angle))
    temporary = tempfile.TemporaryDirectory(prefix="odin_route_")
    config_file = Path(temporary.name) / "route.rviz"
    config_file.write_text(yaml.safe_dump(config))
    rviz = Node(package="rviz2", executable="rviz2", name="odin_route_rviz",
                arguments=["-d", str(config_file)], output="screen",
                additional_env={
                    "LIBGL_ALWAYS_SOFTWARE": "1",
                    "LP_NUM_THREADS": "2",
                    "QT_AUTO_SCREEN_SCALE_FACTOR": "0",
                    "QT_SCALE_FACTOR": "1",
                })
    recorder = Node(
        package="odin_sentry_bringup", executable="record_route.py",
        parameters=[{"output_file": str(output), "map_file": str(map_file)}],
        output="screen")
    return [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             parameters=[{"yaml_filename": str(map_file), "use_sim_time": False}],
             output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="route_map_manager",
             parameters=[{"autostart": True, "node_names": ["map_server"]}]),
        recorder,
        rviz,
        RegisterEventHandler(OnProcessExit(
            target_action=rviz, on_exit=[EmitEvent(event=Shutdown(reason="Route window closed"))])),
        RegisterEventHandler(OnProcessExit(
            target_action=recorder, on_exit=[EmitEvent(event=Shutdown(reason="Recorder exited"))])),
        RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda context: temporary.cleanup())])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", description="Saved occupancy map YAML"),
        DeclareLaunchArgument("output_file", description="New route YAML to save"),
        OpaqueFunction(function=setup),
    ])
