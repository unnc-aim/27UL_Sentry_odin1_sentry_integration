"""Exercise the installed velocity converter and runtime on an isolated ROS domain."""
import os

os.environ["ROS_DOMAIN_ID"] = "113"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import sys
import time
import subprocess
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_prefix
from native_runtime import Runtime
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, TwistStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from example_interfaces.msg import Float32
from action_msgs.srv import CancelGoal

rclpy.init(args=["--ros-args", "-p", "enable_commands:=true"])
runtime = Runtime()
test = rclpy.create_node("runtime_test")
ex = SingleThreadedExecutor()
ex.add_node(runtime)
ex.add_node(test)
static = StaticTransformBroadcaster(test)
dynamic = TransformBroadcaster(test)
anchor = TransformStamped()
anchor.header.frame_id = "odom"
anchor.child_frame_id = "odin_local"
anchor.transform.rotation.w = 1.0
static.sendTransform(anchor)
pubs = {
    name: test.create_publisher(cls, name, 10)
    for name, cls in [
        ("/odin1/tf", TFMessage),
        ("/odin1/odometry", Odometry),
        ("/odometry", Odometry),
        ("/odin_obstacles/lidar_odometry", Odometry),
        ("/odin_3d/terrain_ready", Bool),
        ("/cmd_vel_nav2_result", Twist),
        ("/universal_controller/remote_spin", Float32),
    ]
}
cancel_requests = []


def cancel_goal(request, response):
    cancel_requests.append(request)
    return response


cancel_service = test.create_service(
    CancelGoal, "/navigate_to_pose/_action/cancel_goal", cancel_goal
)
outputs = []
converted = []
sub = test.create_subscription(TwistStamped, "/cmd_vel", outputs.append, 10)
converted_sub = test.create_subscription(
    TwistStamped, "/odin_3d/cmd_vel_checked_input", converted.append, 10
)
params = {
    "robot_base_frame": "gimbal_yaw",
    "fake_robot_base_frame": "gimbal_yaw_fake",
    "odom_topic": "/odometry",
    "input_cmd_vel_topic": "/odin_3d/fvt_input",
    "output_cmd_vel_topic": "/odin_3d/cmd_vel_checked_input",
    "remote_spin_topic": "/universal_controller/remote_spin",
    "cmd_spin_topic": "/cmd_spin",
    "translation_scale": "1.0",
}
command = [
    get_package_prefix("fake_vel_transform")
    + "/lib/fake_vel_transform/fake_vel_transform_node",
    "--ros-args",
]
for key, value in params.items():
    command += ["-p", f"{key}:={value}"]
log = tempfile.TemporaryFile(mode="w+")
proc = subprocess.Popen(command, stdout=log, stderr=log)
tree = None


def run(seconds, raw=True, nav=None, remote=1.2, tree_spin=None, terrain=True, odometry=True):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        assert proc.poll() is None, "Velocity converter exited"
        stamp = test.get_clock().now().to_msg()
        base = TransformStamped()
        base.header.stamp = stamp
        base.header.frame_id = "odom"
        base.child_frame_id = "base_footprint"
        base.transform.rotation.w = 1.0
        dynamic.sendTransform(base)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "gimbal_yaw"
        odom.pose.pose.orientation.w = 1.0
        if odometry:
            pubs["/odin1/odometry"].publish(odom)
            pubs["/odometry"].publish(odom)
        nt = TransformStamped()
        nt.header.stamp = stamp
        nt.header.frame_id = "odom"
        nt.child_frame_id = "map"
        nt.transform.rotation.w = 1.0
        pubs["/odin1/tf"].publish(TFMessage(transforms=[nt]))
        if raw:
            raw_pose = Odometry()
            raw_pose.header.frame_id = "odom"
            raw_pose.header.stamp = stamp
            raw_pose.child_frame_id = "odin_imu"
            pubs["/odin_obstacles/lidar_odometry"].publish(raw_pose)
        pubs["/odin_3d/terrain_ready"].publish(Bool(data=terrain))
        if nav is not None:
            velocity = Twist()
            velocity.linear.x = nav
            pubs["/cmd_vel_nav2_result"].publish(velocity)
        if remote is not None:
            pubs["/universal_controller/remote_spin"].publish(Float32(data=remote))
        if tree is not None and tree_spin is not None:
            tree.publish(Float32(data=tree_spin))
        until = time.monotonic() + 0.04
        while time.monotonic() < until:
            ex.spin_once(timeout_sec=0.002)


def check(x, spin):
    assert len(outputs) >= 4, "Missing final velocity output"
    for msg in outputs[-4:]:
        assert msg.header.frame_id == "gimbal_yaw"
        assert abs(msg.twist.linear.x - x) < 1e-5, (x, msg.twist)
        assert msg.twist.linear.y == msg.twist.linear.z == 0.0
        assert abs(msg.twist.angular.z - spin) < 1e-5, (spin, msg.twist)
    outputs.clear()


try:
    discovery_deadline = time.monotonic() + 6.0
    while time.monotonic() < discovery_deadline:
        run(0.25)
        if (runtime.previous_ready and converted and len(outputs) >= 4
                and all(abs(m.twist.angular.z - 1.2) < 1e-5 for m in outputs[-4:])):
            break
    assert (
        runtime.previous_ready and converted
    ), "Fresh inputs and real converter required"
    check(0.0, 1.2)
    # Runtime receives the paired pose and terrain status; point clouds stay
    # in the perception nodes. Their absence here must preserve command flow.
    run(0.7, nav=0.2)
    check(0.2, 1.2)
    run(0.7)
    check(0.0, 1.2)
    tree = test.create_publisher(Float32, "/cmd_spin", 1)
    run(0.8, tree_spin=0.0)
    check(0.0, 0.0)
    run(0.7, tree_spin=0.8)
    check(0.0, 0.8)
    test.destroy_publisher(tree)
    tree = None
    run(0.8)
    check(0.0, 1.2)
    run(1.1, raw=False, nav=0.2)
    check(0.0, 1.2)
    assert runtime.pause_since is not None and not runtime.motion_fault
    run(0.7, nav=0.2)
    check(0.2, 1.2)
    assert runtime.pause_since is None and runtime.previous_ready
    # Several brief interruptions span more than two seconds in total, but
    # valid input between them must reset the continuous-outage timer.
    for _ in range(4):
        run(0.6, terrain=False, nav=0.2)
        check(0.0, 1.2)
        run(0.08, terrain=True, nav=0.2)
        assert not runtime.motion_fault, "Separate short gaps counted as one outage"
    run(0.7, nav=0.2)
    check(0.2, 1.2)
    run(3.0, raw=False, nav=0.2)
    check(0.0, 1.2)
    assert runtime.pause_cancelled and not runtime.motion_fault
    assert len(cancel_requests) == 1, "Long input loss should cancel the goal once"
    run(0.7, nav=0.2)
    check(0.2, 1.2)
    assert runtime.previous_ready, "Fresh input should permit subsequent navigation"
    # A long pose outage also pauses/cancels, then recovers in the same frame.
    prior_cancels = len(cancel_requests)
    run(3.0, odometry=False, nav=0.2)
    check(0.0, 1.2)
    assert len(cancel_requests) == prior_cancels + 1
    run(0.7, nav=0.2)
    check(0.2, 1.2)
    assert runtime.previous_ready and not runtime.alignment.fault
    runtime.alignment.invalidate()
    run(0.6, nav=0.2)
    check(0.0, 1.2)
    assert runtime.motion_fault, "A changed localization origin still requires restart"
    run(0.6, remote=0.0)
    check(0.0, 0.0)
    run(0.6)
    check(0.0, 1.2)
    run(0.6, remote=None)
    check(0.0, 0.0)
    tree = test.create_publisher(Float32, "/cmd_spin", 1)
    run(0.8, tree_spin=0.0)
    check(0.0, 0.0)
    run(0.6, tree_spin=0.8)
    check(0.0, 0.8)
    run(0.6)
    check(0.0, 0.0)
    run(0.6, remote=float("nan"), tree_spin=0.8)
    check(0.0, 0.0)
    print(
        "Actual converter: idle spin, navigation, BT priority, short-gap recovery, long-gap goal cancellation and recovery, RC zero/loss/invalid and origin-fault restart requirement passed"
    )
except Exception:
    log.seek(0)
    print(log.read())
    raise
finally:
    proc.terminate()
    proc.wait(timeout=5)
    log.close()
    ex.shutdown()
    runtime.destroy_node()
    test.destroy_node()
    rclpy.shutdown()
