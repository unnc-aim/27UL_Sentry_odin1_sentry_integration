#!/usr/bin/env python3
"""Native map alignment and final navigation command validation; never creates goals."""
import copy
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped, TwistStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from example_interfaces.msg import Float32
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from action_msgs.srv import CancelGoal
from scipy.spatial.transform import Rotation
from native_math import Alignment, matrix, close, fresh


class Runtime(Node):
    def __init__(self):
        super().__init__("odin_3d_runtime")
        self.alignment = Alignment()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.br = TransformBroadcaster(self)
        self.odom_stamp = self.raw_stamp = self.terrain_time = self.command_time = 0.0
        self.native_time = 0.0
        self.native_stamp = 0.0
        self.anchor = None
        self.last_odom = None
        self.command = None
        self.nav_command = None
        self.nav_command_time = 0.0
        self.remote_spin = self.tree_spin = 0.0
        self.remote_spin_time = self.tree_spin_time = 0.0
        self.previous_ready = False
        self.motion_fault = False
        self.pause_since = None
        self.invalid_since = None
        self.pause_cancelled = False
        self.recovery_samples = 0
        self.recovery_pair = (0.0, 0.0)
        self.terrain_ok = False
        self.active = bool(self.declare_parameter("enable_commands", False).value)
        self.age = float(self.declare_parameter("max_input_age", 0.5).value)
        self.native_age = float(self.declare_parameter("native_tf_timeout", 2.0).value)
        self.recovery_timeout = float(
            self.declare_parameter("input_recovery_timeout", 2.0).value
        )
        if (
            not all(
                math.isfinite(x) and x > 0
                for x in (self.age, self.native_age, self.recovery_timeout)
            )
            or self.recovery_timeout <= self.age
        ):
            raise ValueError("Invalid input ages")
        self.zero = (
            self.create_publisher(TwistStamped, "/cmd_vel", 5) if self.active else None
        )
        self.nav_input = (
            self.create_publisher(Twist, "/odin_3d/fvt_input", 1)
            if self.active
            else None
        )
        self.create_subscription(
            Twist, "/cmd_vel_nav2_result", self.navigation_input, 1
        )
        self.state = self.create_publisher(String, "/odin_3d/state", 1)
        self.ready = self.create_publisher(Bool, "/odin_3d/ready", 1)
        self.preview = self.create_publisher(PointCloud2, "/odin_3d/color_preview", 1)
        self.color_time = 0.0
        self.create_subscription(TFMessage, "/odin1/tf", self.native, 20)
        self.create_subscription(
            Odometry, "/odin1/odometry", self.odom, qos_profile_sensor_data
        )
        self.create_subscription(
            # The raw processor publishes this pose with the filtered cloud.
            # Observe its measurement stamp without receiving the full cloud again.
            Odometry,
            "/odin_obstacles/lidar_odometry",
            self.raw,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, "/odin_3d/color_filtered", self.color, qos_profile_sensor_data
        )
        self.create_subscription(Bool, "/odin_3d/terrain_ready", self.terrain, 1)
        self.create_subscription(
            TwistStamped, "/odin_3d/cmd_vel_checked_input", self.velocity, 5
        )
        self.create_subscription(
            Float32, "/universal_controller/remote_spin", self.receive_remote_spin, 1
        )
        self.tree_spin_sub = self.create_subscription(
            Float32, "/cmd_spin", self.receive_tree_spin, 1
        )
        self.cancel_clients = [
            self.create_client(CancelGoal, "/" + x + "/_action/cancel_goal")
            for x in ["navigate_to_pose", "navigate_through_poses", "follow_path"]
        ]
        self.pending = []
        self.cancel_needed = False
        self.create_timer(0.05, self.tick)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def odom(self, msg):
        stamp = self.stamp(msg)
        if not fresh(stamp, self.now_seconds(), self.age):
            return
        # Forward gaps use the existing pause/cancel/recovery path in tick().
        # A gap alone does not change the odometry coordinate frame.
        if self.odom_stamp and stamp < self.odom_stamp:
            self.get_logger().error(
                "Device odometry time moved backwards by %.6fs" % (self.odom_stamp - stamp)
            )
            self.alignment.invalidate()
        p = msg.pose.pose
        v = np.array([p.position.x, p.position.y, p.position.z])
        if not np.isfinite(v).all():
            self.alignment.invalidate()
            return
        if self.last_odom is not None and np.linalg.norm(v - self.last_odom) > 1.0:
            self.get_logger().error(
                "Device odometry position changed by %.3fm" % np.linalg.norm(v - self.last_odom)
            )
            self.alignment.invalidate()
        self.last_odom = v
        self.odom_stamp = stamp

    def native(self, msg):
        for t in msg.transforms:
            if t.header.frame_id != "odom" or t.child_frame_id != "map":
                continue
            if self.stamp(t) <= self.native_stamp or not fresh(
                self.stamp(t), self.now_seconds(), self.native_age
            ):
                continue
            try:
                a = matrix(
                    self.buffer.lookup_transform("odom", "odin_local", Time()).transform
                )
                if self.anchor is not None and not close(a, self.anchor, 1e-4, 1e-4):
                    self.alignment.invalidate()
                    return
                self.anchor = a
                self.native_stamp = self.stamp(t)
                self.alignment.observe(
                    np.linalg.inv(matrix(t.transform)) @ np.linalg.inv(a)
                )
                self.native_time = time.monotonic()
            except Exception as e:
                self.get_logger().warning(
                    "Native alignment awaiting valid TF: " + str(e),
                    throttle_duration_sec=3.0,
                )

    def raw(self, msg):
        if (msg.header.frame_id == "odom" and msg.child_frame_id == "odin_imu"
                and self.stamp(msg) > self.raw_stamp):
            self.raw_stamp = self.stamp(msg)

    def terrain(self, msg):
        self.terrain_time = time.monotonic()
        self.terrain_ok = msg.data

    def receive_remote_spin(self, msg):
        self.remote_spin = msg.data if math.isfinite(msg.data) else 0.0
        self.remote_spin_time = time.monotonic()

    def receive_tree_spin(self, msg):
        self.tree_spin = msg.data if math.isfinite(msg.data) else 0.0
        self.tree_spin_time = time.monotonic()

    def current_spin(self, steady):
        # Hub publishes zero for RC loss, emergency stop, disabled navigation or spin.
        if steady - self.remote_spin_time >= 0.2 or self.remote_spin == 0.0:
            return 0.0
        if self.count_publishers("/cmd_spin"):
            return self.tree_spin if steady - self.tree_spin_time < 0.2 else 0.0
        return self.remote_spin

    def navigation_input(self, msg):
        values = [
            getattr(v, a) for v in (msg.linear, msg.angular) for a in ("x", "y", "z")
        ]
        if all(math.isfinite(v) for v in values):
            self.nav_command = msg
            self.nav_command_time = time.monotonic()
        else:
            self.nav_command = None
            self.nav_command_time = 0.0
            if self.active:
                self.motion_fault = True
                self.cancel_needed = True

    def velocity(self, msg):
        values = [
            getattr(v, a)
            for v in (msg.twist.linear, msg.twist.angular)
            for a in ("x", "y", "z")
        ]
        if msg.header.frame_id == "gimbal_yaw" and all(
            math.isfinite(x) for x in values
        ):
            self.command = msg
            self.command_time = time.monotonic()

    def color(self, msg):
        steady = time.monotonic()
        if steady < self.color_time or not fresh(
            self.stamp(msg), self.now_seconds(), self.age
        ):
            return
        if msg.header.frame_id != "odin_local":
            return
        self.color_time = max(self.color_time + 0.1, steady)
        out = copy.deepcopy(msg)
        out.header.frame_id = "odin_local"
        self.preview.publish(out)

    def tick(self):
        steady = time.monotonic()
        now = self.now_seconds()
        if self.nav_input is not None:
            command = Twist()
            if self.nav_command is not None and steady - self.nav_command_time <= 0.15:
                command = self.nav_command
            # Let the existing converter combine idle translation with RC/BT
            # spin, including its heading freshness and explicit BT-zero rules.
            self.nav_input.publish(command)
        inputs = fresh(self.odom_stamp, now, self.age) and fresh(
            self.raw_stamp, now, self.age
        )
        native = (
            self.alignment.pose is not None
            and not self.alignment.fault
            and steady - self.native_time <= self.native_age
        )
        # A live base TF is also required: raw and device poses alone do not prove mount feedback.
        try:
            base = self.buffer.lookup_transform("odom", "base_footprint", Time())
            base_fresh = fresh(self.stamp(base), now, self.age)
        except Exception:
            base_fresh = False
        inputs = inputs and base_fresh
        ok = bool(
            native
            and inputs
            and self.terrain_ok
            and steady - self.terrain_time <= self.age
        )
        if self.active and self.alignment.fault and not self.motion_fault:
            self.motion_fault = True
            self.cancel_needed = True
        if self.active and not self.motion_fault:
            if not ok and (self.previous_ready or self.pause_since is not None):
                if self.pause_since is None:
                    self.pause_since = steady
                    self.pause_cancelled = False
                    self.get_logger().warning(
                        "Navigation input pause: odom=%.3fs raw=%.3fs "
                        "base_fresh=%s native_fresh=%s terrain=%s (%.3fs). "
                        "Translation zero; waiting for three fresh input pairs."
                        % (
                            now - self.odom_stamp,
                            now - self.raw_stamp,
                            base_fresh,
                            native,
                            self.terrain_ok,
                            steady - self.terrain_time,
                        )
                    )
                if self.invalid_since is None:
                    self.invalid_since = steady
                self.recovery_samples = 0
                self.recovery_pair = (self.odom_stamp, self.raw_stamp)
            elif ok:
                # Valid input interrupts the outage, even while three new
                # pairs are still accumulating. Separate gaps stay separate.
                self.invalid_since = None
            if self.pause_since is not None:
                if (
                    self.invalid_since is not None
                    and steady - self.invalid_since > self.recovery_timeout
                    and not self.pause_cancelled
                ):
                    self.pause_cancelled = True
                    self.cancel_needed = True
                    self.get_logger().warning(
                        "Navigation inputs missing for %.1fs; cancelling the current goal. "
                        "Translation stays zero until three fresh input pairs recover."
                        % self.recovery_timeout
                    )
                elif ok:
                    pair = (self.odom_stamp, self.raw_stamp)
                    if all(a > b for a, b in zip(pair, self.recovery_pair)):
                        self.recovery_pair = pair
                        self.recovery_samples += 1
                    if self.recovery_samples >= 3:
                        self.pause_since = None
                        self.get_logger().info(
                            "Navigation inputs recovered after three fresh pairs"
                        )
        ok = ok and not self.motion_fault and self.pause_since is None
        self.previous_ready = ok
        if self.cancel_needed:
            for client in self.cancel_clients:
                if client.service_is_ready():
                    self.pending.append(client.call_async(CancelGoal.Request()))
            self.cancel_needed = False
        self.pending = [f for f in self.pending if not f.done()]
        if native:
            a = self.alignment.pose
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "odom"
            (
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ) = map(float, a[:3, 3])
            q = Rotation.from_matrix(a[:3, :3]).as_quat()
            (
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ) = map(float, q)
            self.br.sendTransform(t)
        self.ready.publish(Bool(data=ok))
        self.state.publish(
            String(
                data="FAULT_RESTART_REQUIRED"
                if self.alignment.fault
                else "TRANSLATION_FAULT_RESTART_REQUIRED"
                if self.motion_fault
                else "WAITING_FRESH_INPUTS"
                if self.pause_since is not None
                else "READY"
                if ok
                else "LOCALIZED_WAITING_PERCEPTION"
                if native
                else "WAITING"
            )
        )
        if self.zero is not None:
            out = TwistStamped()
            out.header.frame_id = "gimbal_yaw"
            out.header.stamp = self.get_clock().now().to_msg()
            if (
                ok
                and self.command is not None
                and steady - self.command_time <= 0.15
                and fresh(self.stamp(self.command), now, 0.15)
            ):
                out.twist = copy.deepcopy(self.command.twist)
            else:
                out.twist.angular.z = self.current_spin(steady)
            self.zero.publish(out)


def main():
    rclpy.init()
    node = Runtime()
    try:
        rclpy.spin(node)
    finally:
        if node.zero is not None:
            msg = TwistStamped()
            msg.header.frame_id = "gimbal_yaw"
            msg.header.stamp = node.get_clock().now().to_msg()
            node.zero.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
