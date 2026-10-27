"""Independent Odin native mapping/localization/navigation; reuse existing nodes."""
import importlib.util
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.frontend.parse_substitution import parse_substitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def setup(context):
    value = lambda k: LaunchConfiguration(k).perform(context)
    mode = value("mode")
    own = Path(share("odin_3d_test"))
    adapter = Path(share("odin_sentry_bringup"))
    nav = Path(share("pb2025_nav_bringup"))
    temp = tempfile.TemporaryDirectory(prefix="odin_3d_")
    path = Path(temp.name)
    spec = importlib.util.spec_from_file_location(
        "odin_existing_launch", adapter / "launch/odin_sentry.launch.py"
    )
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    profile = yaml.safe_load((own / "config/implementation_profile.yaml").read_text())
    flat_ground = profile["added_terrain"].get("ground_mode", "flat") == "flat"
    driver = yaml.safe_load((own / "config/odin_color_mapping.yaml").read_text())
    keys = driver["register_keys"]
    keys["sendrgb"] = int(value("rgb").lower() == "true")
    keys["custom_map_mode"] = 1 if mode == "mapping" else 2
    keys["send_odom_baselink_tf"] = 0
    if mode == "mapping":
        output = Path(value("output")).expanduser().resolve()
        if output.exists():
            raise ValueError("Use a new native map output filename")
        output.parent.mkdir(parents=True, exist_ok=True)
        keys.update(
            mapping_result_dest_dir=str(output.parent),
            mapping_result_file_name=output.name,
        )
    else:
        binary = Path(value("native_map")).expanduser().resolve()
        if not binary.is_file():
            raise ValueError("native_map requires an existing Odin .bin file")
        keys["relocalization_map_abs_path"] = str(binary)
    map_path = Path(value("map")).expanduser().resolve()
    if mode == "navigation" and not map_path.is_file():
        raise ValueError("map requires the matching navigation YAML")
    (path / "driver.yaml").write_text(yaml.safe_dump(driver))
    source = Path(value("params_file")).expanduser()
    if not source.is_file():
        raise ValueError(
            "params_file requires the existing competition Nav2 configuration"
        )
    raw = yaml.safe_load((adapter / "config/raw_obstacles.yaml").read_text())[
        "odin_raw_obstacles"
    ]["ros__parameters"]
    speed_profile = profile["navigation"]
    raw.update(
        max_range=float(speed_profile["raw_range_m"]),
        near_outlier_range=float(speed_profile["near_filter_range_m"]),
    )
    base_params = yaml.safe_load(source.read_text())
    ceiling = float(speed_profile["max_linear_speed_mps"])
    base_params["controller_server"]["ros__parameters"]["FollowPath"].update(
        v_linear_max=ceiling,
        max_lookahead_dist=float(speed_profile["max_lookahead_m"]),
        lookahead_time=0.6,
        translation_kp=4.0,
        # Replanning and obstacle avoidance move the tracking point abruptly.
        # Distance braking and the smoother handle deceleration separately.
        translation_kd=0.0,
    )
    base_params["velocity_smoother"]["ros__parameters"]["max_velocity"][:2] = [ceiling, ceiling]
    base_params["velocity_smoother"]["ros__parameters"]["min_velocity"][:2] = [-ceiling, -ceiling]
    params = helper.navigation_parameters(
        base_params,
        float(value("linear_speed_limit")),
        float(value("angular_speed_limit")),
        raw,
        linear_deceleration=float(speed_profile["linear_deceleration_mps2"]),
    )
    acceleration = float(speed_profile["linear_acceleration_mps2"])
    if not math.isfinite(acceleration) or acceleration <= 0:
        raise ValueError("linear_acceleration_mps2 requires a finite positive value")
    lateral_acceleration = float(speed_profile["lateral_acceleration_mps2"])
    if not math.isfinite(lateral_acceleration) or lateral_acceleration <= 0:
        raise ValueError("lateral_acceleration_mps2 requires a finite positive value")
    params["controller_server"]["ros__parameters"]["FollowPath"][
        "max_lateral_accel"
    ] = lateral_acceleration
    params["velocity_smoother"]["ros__parameters"]["max_accel"][:2] = [
        acceleration, acceleration
    ]
    params["controller_server"]["ros__parameters"]["progress_checker"].update(
        required_movement_radius=0.10,
        movement_time_allowance=3.0,
    )
    params["fake_vel_transform"]["ros__parameters"][
        "output_cmd_vel_topic"
    ] = "/odin_3d/cmd_vel_checked_input"
    params["fake_vel_transform"]["ros__parameters"][
        "input_cmd_vel_topic"
    ] = "/odin_3d/fvt_input"
    # Keep the existing planner and follow-path tree; recover by waiting for
    # observations and replanning instead of issuing a blind retreat.
    for key in ("default_nav_to_pose_bt_xml", "default_nav_through_poses_bt_xml"):
        navigator = params["bt_navigator"]["ros__parameters"]
        source_tree = perform_substitutions(context, parse_substitution(navigator[key]))
        tree = ET.parse(source_tree)
        for action in tree.iter("RemovePassedGoals"):
            action.set("radius", "0.35")
        for recovery in tree.iter("RecoveryNode"):
            if recovery.get("name") == "FollowPath":
                recovery.set("number_of_retries", "2")
        for action in tree.iter("BackUp"):
            action.tag = "Wait"
            action.attrib.clear()
            action.set("wait_duration", "1.0")
        destination = path / (key + ".xml")
        tree.write(destination, encoding="utf-8", xml_declaration=True)
        navigator[key] = str(destination)
    # Planning and following both require surveyed or measured free space.
    params["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] = False
    for name in ["local_costmap", "global_costmap"]:
        cm = params[name][name]["ros__parameters"]
        # Global paths reserve room for local-grid projection and tracking error.
        cm["inflation_layer"]["inflation_radius"] = 0.30 if name == "global_costmap" else 0.25
        if name == "global_costmap":
            cm["footprint_padding"] = 0.10
        if flat_ground:
            # Use the measured floor plane and vehicle clearance for both maps.
            # A separate local-height estimate can treat an arch beam as floor.
            cm["plugins"].remove("intensity_voxel_layer")
            cm.pop("intensity_voxel_layer")
        else:
            layer = cm["intensity_voxel_layer"]
            layer["min_obstacle_height"] = -2.0
            source_name = "terrain_map" if name == "local_costmap" else "terrain_map_ext"
            layer[source_name]["min_obstacle_height"] = -2.0
            layer[source_name]["sensor_frame"] = "odin_obstacle_sensor"
        cm["track_unknown_space"] = True
        cm["inflation_layer"]["inflate_around_unknown"] = True
        cm["plugins"].insert(cm["plugins"].index("inflation_layer"), "surface_layer")
        cm["surface_layer"] = {
            "plugin": "odin_3d_test::SurfaceLayer",
            "enabled": True,
            # Unobserved live ground retains the surveyed static map's costs.
            # Measured terrain hazards still add obstacles before inflation.
            "close_unknown": False,
        }
    (path / "nav.yaml").write_text(yaml.safe_dump(params))
    configured = ParameterFile(str(path / "nav.yaml"), allow_substs=True)
    terrain = dict(params["terrain_analysis"]["ros__parameters"])
    terrain.update(
        decayTime=0.2,
        voxelTimeUpdateThre=0.1,
        noDecayDis=0.0,
        vehicleHeight=profile["robot"]["height_m"],
    )
    actions = helper.mount_nodes(
        yaml.safe_load((adapter / "config/mount.yaml").read_text())
    )
    if mode == "navigation":
        actions.insert(0, SetEnvironmentVariable(
            "FASTRTPS_DEFAULT_PROFILES_FILE", str(own / "config/fastdds_services.xml")
        ))

    def node(pkg, exe, name=None, parameters=None, remappings=None):
        return Node(
            package=pkg,
            executable=exe,
            name=name or exe,
            parameters=parameters or [],
            remappings=remappings or [],
            additional_env={"PYTHONNOUSERSITE": "1", "OPENBLAS_NUM_THREADS": "1"}
            if pkg == "odin_3d_test"
            else {},
            output="screen",
        )

    actions += [
        node(
            "odin_ros_driver",
            "host_sdk_sample",
            parameters=[
                {
                    "config_file": str(path / "driver.yaml"),
                    "ptp_smooth_window": 5,
                    "spad_filter": 1,
                }
            ],
            remappings=[("/tf", "/odin1/tf"), ("/tf_static", "/odin1/tf_static")],
        ),
        node(
            "odin_sentry_bringup",
            "odin_nav_adapter",
            parameters=[str(adapter / "config/self_filter.yaml"),
                        {"independent_odometry": True}],
        ),
        node("odin_sentry_bringup", "odin_raw_obstacles", parameters=[raw]),
        node("odin_3d_test", "color_filter", "odin_color_filter"),
        Node(
            package="terrain_analysis",
            executable="terrainAnalysis",
            name="terrain_analysis",
            namespace="odin_obstacles",
            parameters=[terrain],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(nav / "launch/robot_state_publisher_launch.py")
            ),
            launch_arguments={"use_sim_time": "False"}.items(),
        ),
        node(
            "odin_3d_test",
            "terrain_surface",
            parameters=[
                {
                    "height": float(profile["robot"]["height_m"]),
                    "flat_ground": flat_ground,
                    "obstacle_support": int(profile["added_terrain"].get("obstacle_support", 3)),
                    "clearance_support": int(profile["added_terrain"].get("clearance_support", 6)),
                    "update_rate": float(profile["added_terrain"].get("initial_rate_hz", 10.0)),
                    "clearance": float(profile["robot"]["ground_clearance_m"]),
                    "slope_degrees": float(profile["robot"]["max_slope_degrees"]),
                    "step_height": float(profile["robot"]["step_height_limit_m"]),
                    "window": float(profile["added_terrain"]["initial_window_m"][0]),
                    "resolution": float(
                        profile["added_terrain"]["initial_resolution_m"]
                    ),
                }
            ],
        ),
        node(
            "odin_3d_test",
            "native_runtime.py",
            "odin_3d_runtime",
            parameters=[{"enable_commands": mode == "navigation"}],
        ),
    ]
    if mode == "mapping":
        actions.append(
            node("odin_3d_test", "color_map_display.py", "odin_color_map_display")
        )
    if mode == "navigation":
        actions += [
            node(
                "nav2_map_server",
                "map_server",
                parameters=[{"yaml_filename": str(map_path)}],
            ),
            node(
                "nav2_lifecycle_manager",
                "lifecycle_manager",
                "map_manager",
                parameters=[{"autostart": True, "node_names": ["map_server"]}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav / "launch/navigation_launch.py")),
                launch_arguments={
                    "params_file": str(path / "nav.yaml"),
                    "use_composition": "False",
                    "use_sim_time": "False",
                    "autostart": "True",
                }.items(),
            ),
        ]
    else:
        actions += [
            node(
                "loam_interface", "loam_interface_node", "loam_interface", [configured]
            ),
            node(
                "sensor_scan_generation",
                "sensor_scan_generation_node",
                "sensor_scan_generation",
                [configured],
            ),
        ]
    if value("use_rviz").lower() == "true":
        view = yaml.safe_load((own / "config/view.rviz").read_text())
        for display in view["Visualization Manager"]["Displays"]:
            if display["Name"] == "Navigation Map":
                display["Enabled"] = mode == "navigation"
            if display["Name"] in ("Global Path", "Local Path", "Global Costmap", "Local Costmap"):
                display["Enabled"] = mode == "navigation"
            if display["Name"] == "Accumulated Color Map":
                display["Enabled"] = mode == "mapping"
        (path / "view.rviz").write_text(yaml.safe_dump(view))
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    str(path / "view.rviz"),
                    "-f",
                    "odin_local" if mode == "mapping" else "map",
                ],
                output="screen",
            )
        )
    actions.append(
        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=lambda c: temp.cleanup())])
        )
    )
    return actions


def generate_launch_description():
    defaults = {
        "mode": "localization",
        "rgb": "true",
        "use_rviz": "true",
        "native_map": "",
        "map": "",
        "output": str(Path.home() / "sentry_ws/maps/new_map/native.bin"),
        "params_file": str(
            Path(share("odin_3d_test")) / "config/nav2_competition.yaml"
        ),
        "linear_speed_limit": "4.0",
        "angular_speed_limit": "3.0",
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                k,
                default_value=v,
                **(
                    {"choices": ["mapping", "localization", "navigation"]}
                    if k == "mode"
                    else {}
                )
            )
            for k, v in defaults.items()
        ]
        + [OpaqueFunction(function=setup)]
    )
