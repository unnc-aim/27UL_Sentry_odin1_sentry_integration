import math
import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def vector(config, key):
    value = config[key]
    if (
        not isinstance(value, list)
        or len(value) != 3
        or any(
            isinstance(v, bool)
            or not isinstance(v, (int, float))
            or not math.isfinite(v)
            for v in value
        )
    ):
        raise ValueError(
            f"{key} requires three finite numbers in meters or radians"
        )
    return value


def mount_nodes(config):
    parent = config["parent_frame"]
    if (
        not isinstance(parent, str)
        or not parent
        or parent in ("odin_mount", "odin_imu")
    ):
        raise ValueError("parent_frame must name the robot mounting link")
    nodes = []
    for name, parent, child, xyz, rpy in (
        ("odin_mount_tf", parent, "odin_mount", "mount_xyz", "mount_rpy"),
        (
            "odin_imu_tf",
            "odin_mount",
            "odin_imu",
            "mount_to_imu_xyz",
            "mount_to_imu_rpy",
        ),
    ):
        values = vector(config, xyz) + vector(config, rpy)
        arguments = []
        for flag, value in zip(
            ("--x", "--y", "--z", "--roll", "--pitch", "--yaw"), values
        ):
            arguments.extend((flag, str(value)))
        arguments.extend(("--frame-id", parent, "--child-frame-id", child))
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=name,
                arguments=arguments,
                output="screen",
            )
        )
    return nodes


def navigation_parameters(source, linear_limit, angular_limit, raw_parameters=None):
    if raw_parameters is None:
        share = Path(get_package_share_directory("odin_sentry_bringup"))
        with open(share / "config/raw_obstacles.yaml", encoding="utf-8") as stream:
            raw_parameters = yaml.safe_load(stream)["odin_raw_obstacles"]["ros__parameters"]
    raw_range = raw_parameters["max_range"]
    if not math.isfinite(raw_range) or not 0.15 < raw_range <= 4.0:
        raise ValueError("Raw obstacle range requires 0.15 < max_range <= 4.0 m")
    for value in (linear_limit, angular_limit):
        if not math.isfinite(value) or value <= 0:
            raise ValueError("Speed limits must be finite and positive")
    with open(source, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    description = Path(get_package_share_directory("pb2025_robot_description"))
    with open(
        description / "params/robot_description.yaml", encoding="utf-8"
    ) as stream:
        robot_params = yaml.safe_load(stream)
    params.setdefault(
        "robot_state_publisher", robot_params["robot_state_publisher"]
    )
    params["loam_interface"]["ros__parameters"].update(
        state_estimation_topic="odin_navigation/odometry",
        registered_scan_topic="odin_navigation/cloud",
        odom_frame="odom",
        base_frame="base_footprint",
        lidar_frame="odin_imu",
        align_initial_pose=True,
    )
    params["sensor_scan_generation"]["ros__parameters"].update(
        lidar_frame="odin_imu",
        base_frame="base_footprint",
        robot_base_frame="gimbal_yaw",
        sync_min_interval=0.01,
        sync_max_interval=0.05,
        tf_timeout=0.05,
        max_input_age=0.3,
    )
    for costmap, source_name in (
        ("local_costmap", "terrain_map"),
        ("global_costmap", "terrain_map_ext"),
    ):
        params[costmap][costmap]["ros__parameters"]["inflation_layer"]["inflation_radius"] = (
            0.4 if costmap == "local_costmap" else 0.35
        )
        layer = params[costmap][costmap]["ros__parameters"][
            "intensity_voxel_layer"
        ]
        layer[source_name].update(
            topic="/odin_obstacles/terrain_map",
            sensor_frame="odin_lidar",
            obstacle_max_range=raw_range,
            expected_update_rate=0.5,
            observation_persistence=0.0,
        )
    controller = params["controller_server"]["ros__parameters"]["FollowPath"]
    controller["v_linear_max"] = min(controller["v_linear_max"], linear_limit)
    controller["v_angular_max"] = min(
        controller["v_angular_max"], angular_limit
    )
    controller["v_angular_min"] = max(
        controller["v_angular_min"], -angular_limit
    )
    smoother = params["velocity_smoother"]["ros__parameters"]
    limits = (linear_limit, linear_limit, angular_limit)
    smoother["max_velocity"] = [
        min(v, limit) for v, limit in zip(smoother["max_velocity"], limits)
    ]
    smoother["min_velocity"] = [
        max(v, -limit) for v, limit in zip(smoother["min_velocity"], limits)
    ]
    # Finish a goal after both position and measured translation have settled.
    params["controller_server"]["ros__parameters"]["general_goal_checker"].update(
        plugin="nav2_controller::StoppedGoalChecker",
        stateful=False,
        xy_goal_tolerance=0.1,
        trans_stopped_velocity=0.1,
        # Competition spin and gimbal scanning continue at the destination.
        rot_stopped_velocity=10.0,
    )
    smoother["max_decel"][:2] = [-2.0, -2.0]
    deceleration = min(-float(v) for v in smoother["max_decel"][:2])
    if not math.isfinite(deceleration) or deceleration <= 0:
        raise ValueError("Planar max_decel must contain finite negative accelerations")
    speed = controller["v_linear_max"]
    stopping_distance = speed * speed / (2 * deceleration) + speed * 0.3 + 0.1
    local_map = params["local_costmap"]["local_costmap"]["ros__parameters"]
    footprint = local_map.get("footprint", [])
    if isinstance(footprint, str):
        footprint = yaml.safe_load(footprint) or []
    radius = max(math.hypot(*point) for point in footprint) if footprint else float(local_map.get("robot_radius", 0.2))
    radius += float(local_map.get("footprint_padding", 0.01))
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("A finite positive robot footprint is required for obstacle braking")
    # Keep the robot's swept footprint inside the map throughout the braking range.
    minimum_size = math.ceil(2 * (stopping_distance + radius + 2 * local_map["resolution"]))
    for dimension in ("width", "height"):
        local_map[dimension] = max(local_map[dimension], minimum_size)
    visible_distance = min(local_map["width"], local_map["height"]) / 2 - 2 * local_map["resolution"]
    controller.update(
        obstacle_slowdown_distance=min(visible_distance, raw_range),
        obstacle_stop_margin=0.1,
        obstacle_deceleration=deceleration,
        obstacle_reaction_time=0.3,
        max_lateral_accel=min(float(value) for value in smoother["max_accel"][:2]),
    )
    controller["approach_velocity_scaling_dist"] = max(
        controller["approach_velocity_scaling_dist"], min(stopping_distance, visible_distance)
    )
    # The controller commands zero translation inside the goal position tolerance.
    controller["min_approach_linear_velocity"] = 0.15
    # PB's curve factor recovers gradually after replanning onto a straight path.
    controller["max_velocity_scaling_factor_rate"] = 0.9
    # Retain approach commands in either planar direction.
    smoother["deadband_velocity"][:2] = [0.02, 0.02]
    # A costmap wait pauses controller output. Start braking after three missed
    # 20 Hz commands instead of carrying the last velocity for a full second.
    smoother["velocity_timeout"] = 0.15
    # Apply one acceleration scale to X/Y so diagonal commands retain direction.
    smoother["scale_velocities"] = True
    behaviors = params["behavior_server"]["ros__parameters"]
    behaviors["max_rotational_vel"] = min(
        behaviors.get("max_rotational_vel", angular_limit), angular_limit
    )
    behaviors["min_rotational_vel"] = min(
        behaviors.get("min_rotational_vel", angular_limit), angular_limit
    )
    params["fake_vel_transform"]["ros__parameters"]["init_spin_speed"] = 0.0
    params["fake_vel_transform"]["ros__parameters"]["remote_spin_topic"] = (
        "/universal_controller/remote_spin"
    )
    params["fake_vel_transform"]["ros__parameters"].setdefault(
        "gimbal_imu_topic", "/ecat/sn4653115/app2/read"
    )
    params["fake_vel_transform"]["ros__parameters"].setdefault("max_imu_age", 0.05)
    # slam_toolbox owns map/odom when external odometry is selected.
    # Gimbal sweeps change the view while base_footprint stays stationary.
    # Accept new views by time, with at most five mapping scans per second.
    params["slam_toolbox"]["ros__parameters"].update(
        transform_publish_period=0.05,
        minimum_travel_distance=0.0,
        minimum_travel_heading=0.0,
        minimum_time_interval=0.2,
    )
    params["pointcloud_to_laserscan"]["ros__parameters"].update(
        scan_time=0.1, min_height=0.2
    )
    # Odin mounts where the old Livox MID-360 used to be, so the relocalization
    # node must not keep using the MID-360 frame/extrinsics.
    params.setdefault("small_gicp_relocalization", {}).setdefault(
        "ros__parameters", {}
    )["lidar_frame"] = "odin_imu"
    # Odin supplies SLAM odometry. Chassis spin alone should add only a small
    # pose uncertainty; the previous settings dispersed particles
    # even while the gimbal held its view on the same walls.
    params["amcl"]["ros__parameters"].update(
        max_beams=720, alpha1=0.0001, alpha4=0.0001
    )
    return params


def launch_setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode = value("mode")
    driver_config = value("driver_config")
    with open(driver_config, encoding="utf-8") as stream:
        driver_params = yaml.safe_load(stream)["register_keys"]
    required = {
        "custom_map_mode": 0,
        "use_host_ros_time": 2,
        "sendodom": 1,
        "sendcloudslam": 1,
        "senddtof": 1,
        "send_odom_baselink_tf": 0,
        "tf_extra_publish_rate": 0,
    }
    if any(
        driver_params.get(key) != expected
        for key, expected in required.items()
    ):
        raise ValueError(f"Odin navigation configuration requires {required}")
    actions = [
        Node(
            condition=IfCondition(LaunchConfiguration("start_driver")),
            package="odin_ros_driver",
            executable="host_sdk_sample",
            name="host_sdk_sample",
            parameters=[
                {
                    "config_file": driver_config,
                    "ptp_smooth_window": int(value("ptp_smooth_window")),
                    "spad_filter": int(value("spad_filter")),
                }
            ],
            output="screen",
            remappings=[
                ("/tf", "/odin1/tf"),
                ("/tf_static", "/odin1/tf_static"),
            ],
        )
    ]
    if mode == "driver":
        return actions

    bringup = Path(get_package_share_directory("pb2025_nav_bringup"))
    base_params = value("params_file") or str(
        bringup / "config/reality/nav2_params.yaml"
    )
    map_file = value("map")
    if mode in ("localization", "navigation") and not Path(map_file).is_file():
        raise ValueError(
            "Set map:=<existing occupancy-map YAML> for localization or navigation"
        )
    with open(value("mount_file"), encoding="utf-8") as stream:
        mount = yaml.safe_load(stream)
    actions.extend(mount_nodes(mount))
    actions.append(
        LogInfo(
            msg=f"Odin mount: {mount.get('measurement_reference')}; {mount.get('calibration')}"
        )
    )
    actions.append(
        Node(
            package="odin_sentry_bringup",
            executable="odin_nav_adapter",
            name="odin_nav_adapter",
            output="screen",
            parameters=[
                value("self_filter_file"),
                {
                    name: float(value(name))
                    for name in (
                        "max_stamp_age",
                        "max_future_stamp",
                        "max_pair_dt",
                        "input_timeout",
                        "recovery_timeout",
                        "max_translation_step",
                        "max_rotation_step",
                    )
                },
            ],
        )
    )
    with open(value("raw_obstacles_file"), encoding="utf-8") as stream:
        raw_parameters = yaml.safe_load(stream)["odin_raw_obstacles"]["ros__parameters"]
    params = navigation_parameters(
        base_params,
        float(value("linear_speed_limit")),
        float(value("angular_speed_limit")),
        raw_parameters,
    )
    raw_terrain = dict(params["terrain_analysis"]["ros__parameters"])
    raw_terrain.update(decayTime=0.2, voxelTimeUpdateThre=0.1, noDecayDis=0.0)
    actions.extend([
        Node(
            package="odin_sentry_bringup",
            executable="odin_raw_obstacles",
            name="odin_raw_obstacles",
            parameters=[raw_parameters],
            output="screen",
        ),
        Node(
            package="terrain_analysis",
            executable="terrainAnalysis",
            namespace="odin_obstacles",
            name="terrain_analysis",
            parameters=[raw_terrain],
            output="screen",
        ),
    ])
    temporary = tempfile.TemporaryDirectory(prefix="odin_sentry_")
    params_path = os.path.join(temporary.name, "nav2.yaml")
    with open(params_path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
    actions.append(
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    OpaqueFunction(
                        function=lambda context: temporary.cleanup()
                    )
                ]
            )
        )
    )
    actions.append(
        LogInfo(
            msg=f"Navigation parameters: {base_params}; linear cap {value('linear_speed_limit')} m/s; angular cap {value('angular_speed_limit')} rad/s"
        )
    )

    if mode in ("perception", "localization"):
        actions.append(
            GroupAction(
                [
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            str(
                                bringup
                                / "launch/robot_state_publisher_launch.py"
                            )
                        ),
                        condition=IfCondition(
                            LaunchConfiguration("use_robot_state_pub")
                        ),
                        launch_arguments={
                            "use_sim_time": "False",
                            "params_file": params_path,
                        }.items(),
                    )
                ]
            )
        )
        configured = ParameterFile(params_path, allow_substs=True)
        for package, executable, name in (
            ("loam_interface", "loam_interface_node", "loam_interface"),
            (
                "sensor_scan_generation",
                "sensor_scan_generation_node",
                "sensor_scan_generation",
            ),
            ("terrain_analysis", "terrainAnalysis", "terrain_analysis"),
            (
                "terrain_analysis_ext",
                "terrainAnalysisExt",
                "terrain_analysis_ext",
            ),
            (
                "pointcloud_to_laserscan",
                "pointcloud_to_laserscan_node",
                "pointcloud_to_laserscan",
            ),
        ):
            if mode == "localization" and name == "pointcloud_to_laserscan":
                continue
            remaps = (
                [("cloud_in", "terrain_map_ext"), ("scan", "obstacle_scan")]
                if name == "pointcloud_to_laserscan"
                else []
            )
            actions.append(
                Node(
                    package=package,
                    executable=executable,
                    name=name,
                    parameters=[configured],
                    remappings=remaps,
                    output="screen",
                )
            )
        if mode == "localization":
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(bringup / "launch/localization_launch.py")
                    ),
                    launch_arguments={
                        "use_external_odometry": "True",
                        "use_amcl": "True",
                        "use_composition": "False",
                        "use_sim_time": "False",
                        "map": map_file,
                        "params_file": params_path,
                    }.items(),
                )
            )
            actions.append(
                Node(
                    package="pb2025_nav_bringup",
                    executable="amcl_initializer.py",
                    arguments=[
                        "--globalize",
                        "--method", value("localization_method"),
                        "--seconds", "60",
                    ],
                    parameters=[{"use_sim_time": False}],
                    additional_env={"PYTHONNOUSERSITE": "1", "OPENBLAS_NUM_THREADS": "1"},
                    output="screen",
                )
            )
    else:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(bringup / "launch/rm_navigation_reality_launch.py")
                ),
                launch_arguments={
                    "use_external_odometry": "True",
                    "use_joy_teleop": "False",
                    "slam": str(mode == "mapping"),
                    "map": map_file,
                    "params_file": params_path,
                    "prior_pcd_file": "",
                    "use_robot_state_pub": value("use_robot_state_pub"),
                    "use_rviz": value("use_rviz"),
                    "use_composition": "False",
                    "auto_global_localization": "True",
                    "localization_method": value("localization_method"),
                    "use_sim_time": "False",
                }.items(),
            )
        )
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("odin_sentry_bringup"))
    defaults = {
        "start_driver": "True",
        "use_robot_state_pub": "True",
        "use_rviz": "False",
        "driver_config": str(share / "config/odin_navigation.yaml"),
        "ptp_smooth_window": "1",
        "spad_filter": "1",
        "mount_file": str(share / "config/mount.yaml"),
        "self_filter_file": str(share / "config/self_filter.yaml"),
        "raw_obstacles_file": str(share / "config/raw_obstacles.yaml"),
        "params_file": "",
        "map": "",
        "localization_method": "scan_match",
        "linear_speed_limit": "0.3",
        "angular_speed_limit": "0.5",
        "max_stamp_age": "0.3",
        "max_future_stamp": "0.05",
        "max_pair_dt": "0.05",
        "input_timeout": "0.5",
        "recovery_timeout": "2.0",
        "max_translation_step": "1.0",
        "max_rotation_step": "1.5",
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="driver",
                choices=[
                    "driver",
                    "perception",
                    "localization",
                    "mapping",
                    "navigation",
                ],
            ),
            *(
                DeclareLaunchArgument(name, default_value=value)
                for name, value in defaults.items()
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
