"""Validate configuration and start perception in an isolated domain."""
import math
import os
import runpy
import signal
import subprocess
import sys
import tempfile
import time
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
from launch_ros.utilities import evaluate_parameters


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


def search_example(extra_obstacle=False, reverse_view=False):
    """Generate a limited-view scan in an asymmetric room, with a rotated map origin."""
    import numpy as np
    from sensor_msgs.msg import LaserScan

    grid = np.zeros((160, 200), dtype=np.int8)
    grid[:2, :] = grid[-2:, :] = grid[:, :2] = grid[:, -2:] = 100
    for x0, x1, y0, y1 in ((105, 120, 65, 100), (140, 165, 110, 125),
                          (65, 90, 120, 130), (40, 60, 20, 35)):
        grid[y0:y1, x0:x1] = 100
    if extra_obstacle:
        grid[35:50, 10:30] = 100
    scan = LaserScan()
    scan.header.frame_id = 'base_footprint'
    scan.angle_min, scan.angle_increment = -math.pi, math.pi / 360
    scan.angle_max = scan.angle_min + 719 * scan.angle_increment
    scan.range_min, scan.range_max = .2, 10.0
    ranges = np.full(720, np.inf)
    indices = list(range(120)) + list(range(600, 720)) if reverse_view else range(240, 480)
    for i in indices:
        angle = .7 + scan.angle_min + i * scan.angle_increment
        steps = np.arange(.2, 10.0, .025)
        x = np.floor((2.3 + steps * math.cos(angle)) / .05).astype(int)
        y = np.floor((3.2 + steps * math.sin(angle)) / .05).astype(int)
        inside = (x >= 0) & (x < grid.shape[1]) & (y >= 0) & (y < grid.shape[0])
        hits = np.flatnonzero(grid[y[inside], x[inside]] >= 65)
        if len(hits):
            ranges[i] = steps[inside][hits[0]]
    scan.ranges = ranges.tolist()
    origin = [2.0, -1.0, .4]
    c, s = math.cos(.4), math.sin(.4)
    expected = [2 + c * 2.3 - s * 3.2, -1 + s * 2.3 + c * 3.2, 1.1]
    return grid, scan, origin, expected


def check_global_search():
    import numpy as np
    from threading import Event

    path = Path(get_package_prefix('pb2025_nav_bringup')) / 'lib/pb2025_nav_bringup/scan_map_matcher.py'
    module = runpy.run_path(str(path))
    matcher_type = module['ScanMapMatcher']
    grid, scan, origin, expected = search_example()
    matcher = matcher_type(grid, .05, origin)
    points = matcher.scan_points(scan)
    poses = np.array([expected, [-10, -10, 0], [8, 4, -2], [2, 2, 1]])
    distances = matcher.wall_distances(matcher.endpoints(points, poses))
    direct = np.exp(-0.5 * (distances / .2) ** 2).mean(axis=1)
    direct[~matcher.free(poses[:, :2])] = -1
    assert np.allclose(matcher.endpoint_scores(points, poses), direct, rtol=0, atol=1e-14)
    rows = matcher.search(points, Event())
    assert matcher.accepted(rows), rows[:2]
    assert np.linalg.norm(np.array(rows[0]['pose'][:2]) - expected[:2]) < .15, rows[:2]
    assert abs(rows[0]['pose'][2] - expected[2]) < .06
    repeated = np.tile(grid, (1, 2))
    repeated[35:50, 10:30] = 100
    ambiguous = matcher_type(repeated, .05, origin)
    candidates = ambiguous.search(points, Event())
    assert not ambiguous.accepted(candidates), 'Identical forward views need more information'
    _, reverse_scan, _, _ = search_example(extra_obstacle=True, reverse_view=True)
    observed = ambiguous.observe(
        ambiguous.scan_points(reverse_scan), candidates, [0, 0, 0], [0, 0, 0])
    assert ambiguous.accepted(observed), observed[:2]
    assert np.linalg.norm(np.array(observed[0]['pose'][:2]) - expected[:2]) < .15
    assert observed[0]['score'] <= observed[0]['current_score']
    # Complementary views contribute geometry to the search itself, including
    # a view observed at a different odometry position and heading.
    reverse_points = ambiguous.scan_points(reverse_scan)
    shifted_odom = [.04, -.03, .35]
    c, s = math.cos(shifted_odom[2]), math.sin(shifted_odom[2])
    shifted_points = (reverse_points - shifted_odom[:2]) @ np.array([[c, -s], [s, c]])
    combined, coverage = ambiguous.combine_views(
        [(points, [0, 0, 0]), (shifted_points, shifted_odom)], [0, 0, 0])
    assert len(combined) <= 240 and coverage > 200
    # A repeated view has equal angular weight after combination.
    repeated_views, repeated_coverage = ambiguous.combine_views(
        [(points, [0, 0, 0])] * 8 + [(shifted_points, shifted_odom)], [0, 0, 0])
    assert np.allclose(combined, repeated_views) and coverage == repeated_coverage
    combined_rows = ambiguous.search(combined, Event())
    assert np.linalg.norm(np.array(combined_rows[0]['pose'][:2]) - expected[:2]) < .15
    combined_verified = ambiguous.observe(
        reverse_points, combined_rows, [0, 0, 0], [0, 0, 0])
    assert ambiguous.accepted(combined_verified), combined_verified[:2]
    # A few changed returns can be tolerated; a mostly inconsistent scan still fails.
    changed = points.copy()
    changed[::10] = [25, 25]
    assert matcher.supported(matcher.rank(changed, [expected])[0])
    changed[::2] = [25, 25]
    assert not matcher.supported(matcher.rank(changed, [expected])[0])
    moved = module['move_pose']([4, 5, math.pi / 2], [1, 1, 0], [2, 1, .2])
    assert np.allclose(moved, [4, 6, math.pi / 2 + .2])
    cancel = Event()
    cancel.set()
    try:
        matcher.search(points, cancel)
    except RuntimeError:
        pass
    else:
        raise AssertionError('Search should respond to interruption')
    scan.ranges = [math.inf] * 720
    try:
        matcher.scan_points(scan)
    except ValueError:
        pass
    else:
        raise AssertionError('Empty observations should wait for data')
    check_single_view_initialization(None, method='scan_match')
    print('PASS: unknown pose search, rotated map, repeated-room ambiguity, motion compensation and ROS startup')


def check_single_view_initialization(bringup, method='amcl'):
    """Check that a fresh stationary view starts native global initialization."""
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rclpy.duration import Duration
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import Empty
    from tf2_ros import TransformBroadcaster

    script = (
        Path(get_package_prefix("pb2025_nav_bringup"))
        / "lib/pb2025_nav_bringup/amcl_initializer.py"
    )
    scan = LaserScan()
    scan.header.frame_id = "base_footprint"
    scan.angle_min, scan.angle_increment = -math.pi, math.pi / 360
    scan.range_min, scan.range_max = 0.3, 10.0
    os.environ.update(ROS_DOMAIN_ID="92", ROS_LOCALHOST_ONLY="1")
    rclpy.init()
    node = rclpy.create_node("single_view_initialization_check")
    broadcaster = TransformBroadcaster(node)
    map_pub = node.create_publisher(
        OccupancyGrid, "/map",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    scan_pub = node.create_publisher(LaserScan, "/obstacle_scan", qos_profile_sensor_data)
    calls = []

    def globalize(request, response):
        calls.append(time.monotonic())
        return response

    def active(request, response):
        response.current_state.id = State.PRIMARY_STATE_ACTIVE
        return response

    services = [
        node.create_service(Empty, "/reinitialize_global_localization", globalize),
        node.create_service(Empty, "/request_nomotion_update", lambda request, response: response),
        node.create_service(GetState, "/amcl/get_state", active),
    ]
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = 0.05
    grid.info.width = grid.info.height = 1
    grid.info.origin.orientation.w = 1.0
    grid.data = [0]
    seeds = []
    seed_subscription = node.create_subscription(
        PoseWithCovarianceStamped, '/initialpose', seeds.append, 10)
    view_changed = False
    if method == 'scan_match':
        import numpy as np
        data, scan, origin, expected = search_example()
        data = np.tile(data, (1, 2))
        data[35:50, 10:30] = 100
        _, reverse_scan, _, _ = search_example(extra_obstacle=True, reverse_view=True)
        grid.info.height, grid.info.width = data.shape
        grid.info.origin.position.x, grid.info.origin.position.y = origin[:2]
        grid.info.origin.orientation.z = math.sin(origin[2] / 2)
        grid.info.origin.orientation.w = math.cos(origin[2] / 2)
        grid.data = data.ravel().tolist()
    map_pub.publish(grid)

    def feed(seconds, send_scan=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            now = node.get_clock().now().to_msg()
            transforms = []
            for parent, child in (("odom", "base_footprint"), ("base_footprint", "gimbal_yaw")):
                tf = TransformStamped()
                tf.header.frame_id, tf.child_frame_id, tf.header.stamp = parent, child, now
                if method == 'scan_match':
                    tf.header.stamp = (node.get_clock().now() - Duration(seconds=.1)).to_msg()
                tf.transform.rotation.w = 1.0
                if method == 'scan_match' and view_changed and child == 'gimbal_yaw':
                    tf.transform.rotation.z = 1.0
                    tf.transform.rotation.w = 0.0
                transforms.append(tf)
            broadcaster.sendTransform(transforms)
            if send_scan:
                scan.header.stamp = now
                if method == 'amcl':
                    scan.ranges = [3.0 if 300 <= i < 420 else math.inf for i in range(720)]
                scan_pub.publish(scan)
            until = time.monotonic() + 0.04
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.005)

    with tempfile.TemporaryDirectory(prefix="odin_single_view_test_") as directory:
        report = Path(directory) / "result.json"
        with open(Path(directory) / "node.log", "w+") as log:
            process = subprocess.Popen(
                ["/usr/bin/python3", str(script), "--globalize",
                 "--method", method,
                 "--seconds", "0.3", "--output", str(report)],
                env=dict(os.environ, ROS_LOG_DIR=directory),
                stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 4
                while scan_pub.get_subscription_count() == 0:
                    assert process.poll() is None and time.monotonic() < deadline
                    rclpy.spin_once(node, timeout_sec=0.02)
                feed(0.5, send_scan=False)
                assert not calls and not seeds, "Global initialization started before a scan arrived"
                deadline = time.monotonic() + (25 if method == 'scan_match' else 3)
                while not calls and not seeds and time.monotonic() < deadline:
                    feed(0.1)
                    if method == 'scan_match' and not view_changed and report.exists():
                        import json
                        state = json.loads(report.read_text())
                        if state['latest'].get('reason') == 'Waiting for a distinguishing scan view':
                            assert not seeds, 'Ambiguous first view should keep collecting observations'
                            scan.ranges = reverse_scan.ranges
                            view_changed = True
                if method == 'scan_match':
                    assert view_changed, 'Initialization should wait for the changed view'
                    assert len(seeds) == 1 and not calls, 'Search must initialize once without uniform particles'
                    p = seeds[0].pose.pose.position
                    assert math.hypot(p.x - expected[0], p.y - expected[1]) < .15
                else:
                    assert len(calls) == 1 and not seeds, "A stationary narrow view should initialize once"
                feed(0.5)
                process.wait(timeout=3)
                assert process.returncode == 0
                import json
                result = json.loads(report.read_text())
                assert result['globalization_requested'] == (method == 'amcl')
                if method == 'scan_match':
                    assert result['initial_pose_published']
                    assert result['scan_search']['verification_scans'] >= 3
                    assert len(result['search_attempts']) == 1
                    assert result['scan_search']['source_scans'] >= 1
                    assert result['scan_search']['source_span_s'] <= 10
                assert "scan_collection_seconds" not in result
            except BaseException:
                log.flush()
                log.seek(0)
                print(log.read()[-3000:])
                raise
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                node.destroy_node()
                rclpy.shutdown()


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
    assert raw["near_outlier_range"] == raw["max_range"] == 3.0
    assert raw["near_outlier_radius"] == 0.04
    assert raw["near_outlier_min_neighbors"] == 20
    assert raw["near_outlier_scale_by_range"] is True
    original = source.read_text()
    parameters = module["navigation_parameters"](source, 0.3, 0.5)
    assert source.read_text() == original
    goal_checker = parameters['controller_server']['ros__parameters']['general_goal_checker']
    assert goal_checker['plugin'] == 'nav2_controller::StoppedGoalChecker'
    assert goal_checker['stateful'] is False
    assert goal_checker['xy_goal_tolerance'] == 0.1
    assert goal_checker['trans_stopped_velocity'] == 0.1
    assert goal_checker['rot_stopped_velocity'] == 10.0
    assert parameters['velocity_smoother']['ros__parameters']['deadband_velocity'][:2] == [0.02, 0.02]
    assert parameters['velocity_smoother']['ros__parameters']['velocity_timeout'] == 0.15
    assert parameters['velocity_smoother']['ros__parameters']['scale_velocities'] is True
    assert parameters['velocity_smoother']['ros__parameters']['max_decel'][:2] == [-2.0, -2.0]
    assert parameters['velocity_smoother']['ros__parameters']['max_decel'][2] == yaml.safe_load(original)['velocity_smoother']['ros__parameters']['max_decel'][2]
    follow = parameters['controller_server']['ros__parameters']['FollowPath']
    assert follow['obstacle_deceleration'] == 2.0
    assert follow['obstacle_stop_margin'] == 0.1
    assert follow['obstacle_reaction_time'] == 0.3
    assert follow['max_lateral_accel'] == min(parameters['velocity_smoother']['ros__parameters']['max_accel'][:2])
    local_map = parameters['local_costmap']['local_costmap']['ros__parameters']
    assert follow['obstacle_slowdown_distance'] == min(raw['max_range'], min(local_map['width'], local_map['height']) / 2 - 2 * local_map['resolution'])
    with tempfile.TemporaryDirectory() as directory:
        fast = yaml.safe_load(original)
        fast['controller_server']['ros__parameters']['FollowPath']['v_linear_max'] = 2.5
        fast_source = Path(directory) / 'fast.yaml'
        fast_source.write_text(yaml.safe_dump(fast))
        fast_output = module['navigation_parameters'](fast_source, 3.0, 3.0)
        fast_local = fast_output['local_costmap']['local_costmap']['ros__parameters']
        fast_follow = fast_output['controller_server']['ros__parameters']['FollowPath']
        assert min(fast_local['width'], fast_local['height']) >= 6
        assert fast_follow['v_linear_max'] == 2.5
        assert fast_follow['obstacle_slowdown_distance'] <= raw['max_range']
        fast['controller_server']['ros__parameters']['FollowPath']['v_linear_max'] = 5.0
        fast['velocity_smoother']['ros__parameters']['max_velocity'][:2] = [5.0, 5.0]
        fast['velocity_smoother']['ros__parameters']['min_velocity'][:2] = [-5.0, -5.0]
        saved_input = yaml.safe_dump(fast)
        wide = module['navigation_parameters'](
            fast, 5.0, 3.0, {'max_range': 10.0}, linear_deceleration=2.5)
        wide_follow = wide['controller_server']['ros__parameters']['FollowPath']
        wide_local = wide['local_costmap']['local_costmap']['ros__parameters']
        assert yaml.safe_dump(fast) == saved_input
        assert wide_follow['v_linear_max'] == 5.0
        assert abs(wide_follow['approach_velocity_scaling_dist'] - 6.6) < 1e-6
        assert wide_follow['obstacle_slowdown_distance'] >= 6.6
        assert wide_follow['obstacle_deceleration'] == 2.5
        assert min(wide_local['width'], wide_local['height']) >= 14
        assert wide['velocity_smoother']['ros__parameters']['max_decel'][:2] == [-2.5, -2.5]
    assert parameters['controller_server']['ros__parameters']['FollowPath']['max_velocity_scaling_factor_rate'] == 0.9
    assert parameters['controller_server']['ros__parameters']['FollowPath']['min_approach_linear_velocity'] == 0.15
    assert parameters['fake_vel_transform']['ros__parameters']['gimbal_imu_topic'] == '/ecat/sn4653115/app2/read'
    assert parameters['fake_vel_transform']['ros__parameters']['remote_spin_topic'] == '/universal_controller/remote_spin'
    assert parameters['fake_vel_transform']['ros__parameters']['max_imu_age'] == 0.05
    assert parameters['amcl']['ros__parameters']['alpha1'] == 0.0001
    assert parameters['amcl']['ros__parameters']['alpha4'] == 0.0001
    for name in ('alpha2', 'alpha3', 'alpha5'):
        assert parameters['amcl']['ros__parameters'][name] == yaml.safe_load(original)['amcl']['ros__parameters'][name]
    for name, source_name in (("local_costmap", "terrain_map"), ("global_costmap", "terrain_map_ext")):
        assert parameters[name][name]["ros__parameters"]["inflation_layer"]["inflation_radius"] == (0.4 if name == "local_costmap" else 0.35)
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
    scan = parameters["sensor_scan_generation"]["ros__parameters"]
    assert scan["sync_max_interval"] == 0.05
    assert scan["tf_timeout"] == 0.05
    assert scan["max_input_age"] == 0.3
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
    for value in (0.1, 30.1, math.nan):
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
        else:
            raw_terrain = next(
                a for a in actions
                if isinstance(a, Node) and package(a, context) == "terrain_analysis"
            )
            raw_terrain_values = evaluate_parameters(
                context, raw_terrain._Node__parameters
            )[0]
            assert raw_terrain_values["decayTime"] == 0.2
            assert raw_terrain_values["voxelTimeUpdateThre"] == 0.1
            assert raw_terrain_values["clearDyObs"] == yaml.safe_load(original)[
                "terrain_analysis"]["ros__parameters"]["clearDyObs"]
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
            assert arguments['localization_method'] == 'scan_match'
            assert "localization_collect_seconds" not in arguments
            assert "localization_min_coverage_degrees" not in arguments
        if mode in ("localization", "navigation", "mapping"):
            generated = yaml.safe_load(Path(arguments["params_file"]).read_text())
            for name in ("terrain_analysis", "terrain_analysis_ext"):
                terrain = generated[name]["ros__parameters"]
                assert terrain == yaml.safe_load(original)[name]["ros__parameters"]

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
    check_single_view_initialization(bringup)
    print(
        "PASS: launch modes, independent raw terrain and stationary native initialization"
    )


if __name__ == "__main__":
    if '--global-search-check' in sys.argv:
        check_global_search()
    else:
        main()
