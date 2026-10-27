import importlib.util, tempfile
import yaml
import xml.etree.ElementTree as ET
from pathlib import Path
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.utilities import perform_substitutions

p = Path(__file__).parents[1] / "launch/odin_3d.launch.py"
spec = importlib.util.spec_from_file_location("new_launch", p)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
with tempfile.TemporaryDirectory() as root:
    binary = Path(root) / "map.bin"
    binary.write_bytes(b"test")
    (Path(root) / "navigation.yaml").write_text("image: navigation.pgm\n")
    for mode in ["mapping", "localization", "navigation"]:
        ctx = LaunchContext()
        ctx.launch_configurations.update(
            mode=mode,
            rgb="true",
            use_rviz="true",
            native_map=str(binary),
            map=str(Path(root) / "navigation.yaml"),
            output=root + "/new.bin",
            params_file=str(p.parents[1] / "config/nav2_competition.yaml"),
            linear_speed_limit="4.0",
            angular_speed_limit="3.0",
        )
        actions = m.setup(ctx)
        assert len(actions) > 8
        if mode == "navigation":
            environment = next(a for a in actions if isinstance(a, SetEnvironmentVariable))
            environment.execute(ctx)
            transport = ET.parse(ctx.environment["FASTRTPS_DEFAULT_PROFILES_FILE"])
            ns = {"dds": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
            profiles = transport.findall("dds:publisher", ns)
            assert len(profiles) == 1 and profiles[0].get("profile_name") == "service"
            assert profiles[0].get("is_default_profile", "false") == "false"
            assert profiles[0].find("dds:qos/dds:reliability/dds:max_blocking_time/dds:sec", ns).text == "2"
            for action in actions:
                if isinstance(action, IncludeLaunchDescription):
                    args = dict(action.launch_arguments)
                    if "params_file" in args:
                        config = yaml.safe_load(Path(args["params_file"]).read_text())
                        controller = config["controller_server"]["ros__parameters"]
                        assert controller["progress_checker"]["movement_time_allowance"] == 3.0
                        assert controller["progress_checker"]["required_movement_radius"] == .10
                        assert controller["failure_tolerance"] == .3
                        follow = controller["FollowPath"]
                        assert follow["v_linear_max"] == 4.0
                        assert follow["max_lookahead_dist"] == 1.0
                        assert follow["lookahead_time"] == 0.6
                        assert follow["translation_kp"] == 4.0
                        assert follow["translation_kd"] == 0.0
                        assert abs(follow["approach_velocity_scaling_dist"] - 4.5) < 1e-6
                        assert follow["obstacle_slowdown_distance"] >= 4.5
                        assert follow["obstacle_deceleration"] == 2.5
                        smooth = config["velocity_smoother"]["ros__parameters"]
                        assert smooth["max_velocity"][:2] == [4.0, 4.0]
                        assert smooth["min_velocity"][:2] == [-4.0, -4.0]
                        assert smooth["max_decel"][:2] == [-2.5, -2.5]
                        assert smooth["max_accel"] == [2.5, 2.5, 1.5]
                        assert follow["max_lateral_accel"] == 2.5
                        assert min(config["local_costmap"]["local_costmap"]["ros__parameters"][k]
                                   for k in ["width", "height"]) >= 10
                        for key in ("default_nav_to_pose_bt_xml", "default_nav_through_poses_bt_xml"):
                            tree = ET.parse(config["bt_navigator"]["ros__parameters"][key])
                            assert not list(tree.iter("BackUp"))
                            if "through_poses" in key:
                                assert [a.get("radius") for a in tree.iter("RemovePassedGoals")] == ["0.35"]
                            assert any(a.get("wait_duration") == "1.0" for a in tree.iter("Wait"))
                            assert any(a.get("hz") == "3.0" for a in tree.iter("RateController"))
                            recovery = {a.get("name"): a.get("number_of_retries")
                                        for a in tree.iter("RecoveryNode")}
                            assert recovery["FollowPath"] == "2"
                            assert recovery["NavigateRecovery"] == "10"
                        assert config["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] is False
                        for name in ("local_costmap", "global_costmap"):
                            cm = config[name][name]["ros__parameters"]
                            assert cm["track_unknown_space"] is True
                            assert cm["inflation_layer"]["inflate_around_unknown"] is True
                            assert cm["inflation_layer"]["inflation_radius"] == (
                                .30 if name == "global_costmap" else .25)
                            assert cm["robot_radius"] == .2
                            if name == "global_costmap":
                                assert cm["footprint_padding"] == .10
                            assert cm["plugins"] == ["static_layer", "surface_layer", "inflation_layer"]
                            assert "intensity_voxel_layer" not in cm
            rviz = next(a for a in actions if isinstance(a, Node) and a.node_executable == "rviz2")
            cmd = [perform_substitutions(ctx, part) for part in rviz.cmd]
            view = yaml.safe_load(Path(cmd[cmd.index("-d") + 1]).read_text())
            paths = {d["Topic"]["Value"] for d in view["Visualization Manager"]["Displays"]
                     if d["Class"] == "rviz_default_plugins/Path" and d["Enabled"]}
            assert paths == {"/plan", "/local_plan"}
        from launch_ros.utilities import evaluate_parameters
        for action in actions:
            if isinstance(action, Node) and action.node_executable in ("odin_raw_obstacles", "terrain_surface"):
                values = evaluate_parameters(ctx, action._Node__parameters)[0]
                if action.node_executable == "odin_raw_obstacles":
                    assert values["max_range"] == 10.0
                    assert values["near_outlier_range"] == 4.0
                else:
                    assert values["window"] == 18.0
                    assert values["resolution"] == .05 and values["update_rate"] == 10.0
        filter_nodes = [
            action
            for action in actions
            if isinstance(action, Node) and action.node_executable == "color_filter"
        ]
        assert (
            len(filter_nodes) == 1
        ), "All modes include the existing geometry filter before RGB display"

        environments = [
            dict(
                (perform_substitutions(ctx, k), perform_substitutions(ctx, v))
                for k, v in action.additional_env or []
            )
            for action in actions
            if isinstance(action, Node)
        ]
        assert sum(e.get("PYTHONNOUSERSITE") == "1" for e in environments) >= 2

print("All three modes construct successfully without starting robot nodes")
