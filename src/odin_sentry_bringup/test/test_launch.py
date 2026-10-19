"""Validate configuration and start perception in an isolated domain."""
import math
import os
import runpy
import subprocess
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node


def defaults(description):
    context = LaunchContext()
    # Required maps are supplied explicitly by this check.
    context.launch_configurations["map"] = "/tmp/odin_test_map.yaml"
    context.launch_configurations["prior_pcd_file"] = ""
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def package(node, context):
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(node.node_package)
    )


def main():
    share = Path(get_package_share_directory("odin_sentry_bringup"))
    bringup = Path(get_package_share_directory("pb2025_nav_bringup"))
    driver = (
        Path(get_package_prefix("odin_ros_driver"))
        / "lib/odin_ros_driver/host_sdk_sample"
    )
    with tempfile.TemporaryDirectory(prefix="odin_config_test_") as directory:
        missing = str(Path(directory) / "missing.yaml")
        environment = os.environ.copy()
        environment.update(
            ROS_DOMAIN_ID="87", ROS_LOCALHOST_ONLY="1", ROS_LOG_DIR=directory
        )
        result = subprocess.run(
            [
                str(driver),
                "--ros-args",
                "-p",
                "config_file:=" + missing,
                "-p",
                "ptp_smooth_window:=1",
            ],
            capture_output=True,
            text=True,
            timeout=5,
            env=environment,
        )
        assert result.returncode != 0
        assert (
            "Loading config file: " + missing in result.stdout + result.stderr
        )
        assert (
            "PTP smoothing window: 1 samples" in result.stdout + result.stderr
        )
        for window in (0, 301):
            result = subprocess.run(
                [
                    str(driver),
                    "--ros-args",
                    "-p",
                    "config_file:=" + missing,
                    "-p",
                    f"ptp_smooth_window:={window}",
                ],
                capture_output=True,
                text=True,
                timeout=5,
                env=environment,
            )
            assert result.returncode != 0
            assert (
                "ptp_smooth_window requires 1 to 300 samples"
                in result.stdout + result.stderr
            )
        for spad in (-2, 2):
            result = subprocess.run(
                [str(driver), "--ros-args", "-p", "config_file:=" + missing,
                 "-p", f"spad_filter:={spad}"],
                capture_output=True, text=True, timeout=5, env=environment,
            )
            assert result.returncode != 0
            assert "spad_filter requires -1, 0 or 1" in result.stdout + result.stderr
    module = runpy.run_path(str(share / "launch/odin_sentry.launch.py"))
    source = bringup / "config/reality/nav2_params.yaml"
    raw = yaml.safe_load((share / "config/raw_obstacles.yaml").read_text())["odin_raw_obstacles"]["ros__parameters"]
    assert raw["near_outlier_range"] == 1.0
    assert raw["near_outlier_radius"] == 0.04
    assert raw["near_outlier_min_neighbors"] == 5
    original = source.read_text()
    parameters = module["navigation_parameters"](source, 0.3, 0.5)
    assert source.read_text() == original
    for name, source_name in (("local_costmap", "terrain_map"), ("global_costmap", "terrain_map_ext")):
        layer = parameters[name][name]["ros__parameters"]["intensity_voxel_layer"]
        assert layer[source_name]["topic"] == "/odin_obstacles/terrain_map"
        assert layer[source_name]["sensor_frame"] == "odin_lidar"
        assert layer[source_name]["expected_update_rate"] == 0.5
        assert layer[source_name]["obstacle_max_range"] == 3.0
    assert (
        parameters["loam_interface"]["ros__parameters"]["lidar_frame"]
        == "odin_imu"
    )
    assert (
        parameters["loam_interface"]["ros__parameters"]["align_initial_pose"]
        is True
    )
    assert (
        parameters["sensor_scan_generation"]["ros__parameters"][
            "robot_base_frame"
        ]
        == "gimbal_yaw"
    )
    assert (
        parameters["sensor_scan_generation"]["ros__parameters"][
            "sync_min_interval"
        ]
        == 0.01
    )
    assert (
        parameters["controller_server"]["ros__parameters"]["FollowPath"][
            "v_linear_max"
        ]
        == 0.3
    )
    assert parameters["velocity_smoother"]["ros__parameters"][
        "max_velocity"
    ] == [0.3, 0.3, 0.5]
    assert (
        parameters["fake_vel_transform"]["ros__parameters"][
            "translation_scale"
        ]
        == yaml.safe_load(original)["fake_vel_transform"]["ros__parameters"][
            "translation_scale"
        ]
    )
    assert (
        parameters["slam_toolbox"]["ros__parameters"][
            "transform_publish_period"
        ]
        > 0
    )
    slam = parameters["slam_toolbox"]["ros__parameters"]
    assert slam["minimum_travel_distance"] == 0.0
    assert slam["minimum_travel_heading"] == 0.0
    assert slam["minimum_time_interval"] == 0.2
    for value in (0.0, -1.0, math.nan):
        try:
            module["navigation_parameters"](source, value, 0.5)
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid speed limit accepted")
    for value in (0.1, 5.0, math.nan):
        try:
            module["navigation_parameters"](source, 0.3, 0.5, {"max_range": value})
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid raw terrain range accepted")
    mount = yaml.safe_load((share / "config/mount.yaml").read_text())
    assert mount["parent_frame"] == "gimbal_yaw"
    assert mount["mount_xyz"] == [0.13, 0.07, 0.327]
    assert len(module["mount_nodes"](mount)) == 2
    map_file = bringup / "map/reality/field_20260925.yaml"
    for mode in (
        "driver",
        "perception",
        "localization",
        "navigation",
        "mapping",
    ):
        context = defaults(module["generate_launch_description"]())
        context.launch_configurations.update(
            mode=mode, map=str(map_file), start_driver="False"
        )
        actions = module["launch_setup"](context)
        packages = [
            package(a, context) for a in actions if isinstance(a, Node)
        ]
        if mode == "driver":
            assert packages == ["odin_ros_driver"]
        if mode in ("perception", "localization"):
            assert "sensor_scan_generation" in packages
            assert (
                "fake_vel_transform" not in packages
                and "nav2_controller" not in packages
            )
        if mode == "localization":
            include = next(
                a for a in actions if isinstance(a, IncludeLaunchDescription)
            )
            arguments = dict(include.launch_arguments)
            assert arguments["use_external_odometry"] == "True"
            assert arguments["use_amcl"] == "True"
            assert "pointcloud_to_laserscan" not in packages
        if mode in ("navigation", "mapping"):
            include = next(
                a for a in actions if isinstance(a, IncludeLaunchDescription)
            )
            arguments = dict(include.launch_arguments)
            assert arguments["use_external_odometry"] == "True"
            assert arguments["use_joy_teleop"] == "False"
            assert arguments["slam"] == str(mode == "mapping")

    # Check both the original sensor path and the external odometry path.
    for filename, expected_package in (
        ("rm_navigation_reality_launch.py", "livox_ros_driver2"),
        ("slam_launch.py", "point_lio"),
        ("localization_launch.py", "point_lio"),
    ):
        entry = runpy.run_path(str(bringup / "launch" / filename))
        description = entry["generate_launch_description"]()
        context = defaults(description)
        for external, expected_enabled in (("False", True), ("True", False)):
            context.launch_configurations["use_external_odometry"] = external
            actions = (
                entry["launch_localization"](context)
                if "launch_localization" in entry
                else description.entities
            )
            node = next(
                a
                for a in actions
                if isinstance(a, Node)
                and package(a, context) == expected_package
            )
            assert node.condition.evaluate(context) == expected_enabled
            if filename == "slam_launch.py":
                static = next(
                    a
                    for a in actions
                    if isinstance(a, Node) and package(a, context) == "tf2_ros"
                )
                assert static.condition.evaluate(context) == expected_enabled
    for mode, expected_count in (("perception", 11), ("localization", 15)):
        starts = []
        errors = []

        def exited(event, context):
            if event.returncode != 0 and not context.is_shutdown:
                errors.append(event.returncode)

        directory = tempfile.TemporaryDirectory(prefix="odin_launch_test_")
        service = LaunchService()
        service.include_launch_description(
            LaunchDescription(
                [
                    SetEnvironmentVariable("ROS_DOMAIN_ID", "89"),
                    SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
                    SetEnvironmentVariable("ROS_LOG_DIR", directory.name),
                    SetLaunchConfiguration("mode", mode),
                    SetLaunchConfiguration("map", str(map_file)),
                    SetLaunchConfiguration("start_driver", "False"),
                    RegisterEventHandler(
                        OnProcessStart(
                            on_start=lambda event, context: starts.append(
                                event
                            )
                        )
                    ),
                    RegisterEventHandler(OnProcessExit(on_exit=exited)),
                    *module["generate_launch_description"]().entities,
                    TimerAction(
                        period=3.0,
                        actions=[
                            EmitEvent(
                                event=Shutdown(reason="Startup check complete")
                            )
                        ],
                    ),
                ]
            )
        )
        assert service.run() == 0
        assert len(starts) == expected_count, (mode, len(starts))
        assert errors == [], errors
        directory.cleanup()
    print(
        "PASS: mount, speed limits, source preservation, five modes and perception/localization startup"
    )


if __name__ == "__main__":
    main()
