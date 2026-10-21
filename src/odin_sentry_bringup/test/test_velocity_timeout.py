"""Run the native velocity smoother with an interrupted controller stream."""
import json
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter


def main():
    os.environ.update(ROS_DOMAIN_ID='95', ROS_LOCALHOST_ONLY='1')
    rclpy.init()
    node = rclpy.create_node('velocity_timeout_check')
    pub = node.create_publisher(Twist, '/timeout_input', 10)
    samples = []
    vectors = []

    def receive(msg):
        now = time.monotonic()
        samples.append((now, msg.linear.x))
        vectors.append((now, msg.linear.x, msg.linear.y))

    node.create_subscription(Twist, '/timeout_output', receive, 10)

    def wait(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.002)

    def result(future):
        end = time.monotonic() + 5
        while not future.done():
            wait(.01)
            assert time.monotonic() < end
        return future.result()

    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([
            get_package_prefix('nav2_velocity_smoother') + '/lib/nav2_velocity_smoother/velocity_smoother', '--ros-args',
            '-r', 'cmd_vel:=/timeout_input', '-r', 'cmd_vel_smoothed:=/timeout_output',
            '-p', 'smoothing_frequency:=20.0', '-p', 'max_velocity:=[3.0,3.0,3.0]',
            '-p', 'min_velocity:=[-3.0,-3.0,-3.0]', '-p', 'max_accel:=[1.5,1.5,1.5]',
            '-p', 'max_decel:=[-2.0,-2.0,-1.5]', '-p', 'feedback:=OPEN_LOOP',
            '-p', 'scale_velocities:=true',
            '-p', 'bond_heartbeat_period:=0.0'], stdout=log, stderr=subprocess.STDOUT)
        try:
            lifecycle = node.create_client(ChangeState, '/velocity_smoother/change_state')
            assert lifecycle.wait_for_service(timeout_sec=5)
            for state in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                assert result(lifecycle.call_async(ChangeState.Request(
                    transition=Transition(id=state)))).success
            change = node.create_client(SetParametersAtomically, '/velocity_smoother/set_parameters_atomically')
            assert change.wait_for_service(timeout_sec=3)
            outcomes = {}
            for timeout in (1.0, .15):
                assert result(change.call_async(SetParametersAtomically.Request(parameters=[
                    Parameter('velocity_timeout', value=timeout).to_parameter_msg()
                ]))).result.successful
                command = Twist()
                command.linear.x = 1.26
                end = time.monotonic() + 1.3
                while time.monotonic() < end:
                    pub.publish(command)
                    wait(.04)
                assert samples[-1][1] > 1.25, 'Normal navigation speed must be preserved'
                last = time.monotonic()
                pub.publish(command)
                samples.clear()
                wait(2.2)
                braking = next(t for t, speed in samples if speed < 1.20) - last
                stop = next(t for t, speed in samples if abs(speed) < .001) - last
                distance = sum((b[0] - a[0]) * (a[1] + b[1]) / 2
                               for a, b in zip(samples, samples[1:]))
                outcomes[str(timeout)] = dict(braking_s=braking, stop_s=stop,
                                              command_distance_m=distance)
                assert abs(samples[-1][1]) < .001
                wait(.1)
            assert .90 < outcomes['1.0']['braking_s'] < 1.2
            assert .10 < outcomes['0.15']['braking_s'] < .30
            braking_duration = outcomes['0.15']['stop_s'] - outcomes['0.15']['braking_s']
            assert .45 < braking_duration < .75, braking_duration
            assert outcomes['0.15']['command_distance_m'] < outcomes['1.0']['command_distance_m'] - .8
            # X/Y share an acceleration scale during an obstacle stop and restart.
            vectors.clear()
            for x, y in ((1.26, .63), (0., 0.), (-.6, .3)):
                command = Twist()
                command.linear.x, command.linear.y = x, y
                end = time.monotonic() + 1.1
                while time.monotonic() < end:
                    pub.publish(command)
                    wait(.04)
                assert abs(vectors[-1][1] - x) < .001
                assert abs(vectors[-1][2] - y) < .001
            forward = [(x, y) for _, x, y in vectors if x > .01]
            reverse = [(x, y) for _, x, y in vectors if x < -.01]
            assert len(forward) > 10 and len(reverse) > 5
            assert all(abs(x - 2 * y) < 1e-6 for x, y in forward)
            assert all(abs(x + 2 * y) < 1e-6 for x, y in reverse)
            print(json.dumps(outcomes))
            print('PASS: native smoother expiry, earlier braking, diagonal direction, obstacle stop and restart')
        except BaseException:
            log.seek(0)
            print(log.read()[-2500:])
            raise
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
