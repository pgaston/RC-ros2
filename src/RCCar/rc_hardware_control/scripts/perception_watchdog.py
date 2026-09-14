#!/usr/bin/env python3
"""Perception watchdog: runs the perception bring-up, holds the car while it is unhealthy, restarts it when it stalls.

The camera driver reconnects only on a USB remove or add event: a stream that
stops with the device still on the bus is never noticed, the node did not
recover after a `usbreset`, and a container whose camera never appears
segfaults (issue #14). So this node owns the bring-up as a child process
(`command`, by default `ros2 launch rc_hardware_control perception.launch.py`)
and watches the streams Nav2 depends on: depth, visual SLAM odometry and the
nvblox occupancy grid.

While any of them is stale, or the bring-up is down, it publishes zero on the
velocity mux's hold source, which outranks Nav2 and yields to teleop: the car
stops and a human can still drive it. A stream that stalls, or a bring-up that
exits, gets Nav2's Goals cancelled and the whole bring-up stopped (SIGINT, then
SIGKILL) and started again. Every state change goes to the status topic as
'<state>: <reason>', the states being starting, healthy, stale, restarting and down.

Parameters (see config/perception_watchdog.yaml for the values and why):
  command, hold_topic, status_topic, tick_rate_hz, stop_timeout_s,
  restart_after_s, startup_grace_s, restart_delay_s, cancel_actions,
  streams (list of names), and per stream <name>.topic, <name>.type, <name>.timeout_s.
"""
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

from rc_hardware_control.perception_watchdog_policy import Stream, Timing, WatchdogPolicy
from rc_hardware_control.supervised_process import SupervisedProcess

# name: (topic, type, timeout_s)
DEFAULT_STREAMS = {
    'depth': ('/depth/camera_info', 'sensor_msgs/msg/CameraInfo', 0.5),
    'odometry': ('/visual_slam/tracking/odometry', 'nav_msgs/msg/Odometry', 0.5),
    'occupancy_grid': ('/nvblox_node/static_occupancy_grid', 'nav_msgs/msg/OccupancyGrid', 2.0),
}

# On our own exit: launch sends SIGINT and escalates to SIGTERM after 5 s, so
# the bring-up gets SIGKILL before that.
SHUTDOWN_TIMEOUT_S = 4.0


class PerceptionWatchdog(Node):
    def __init__(self):
        super().__init__('perception_watchdog')
        self.declare_parameter(
            'command', ['ros2', 'launch', 'rc_hardware_control', 'perception.launch.py'])
        self.declare_parameter('hold_topic', '/cmd_vel_hold')
        self.declare_parameter('status_topic', '/perception/status')
        self.declare_parameter('tick_rate_hz', 20.0)
        self.declare_parameter('stop_timeout_s', 10.0)
        defaults = Timing()
        self.declare_parameter('restart_after_s', defaults.restart_after_s)
        self.declare_parameter('startup_grace_s', defaults.startup_grace_s)
        self.declare_parameter('restart_delay_s', defaults.restart_delay_s)
        self.declare_parameter('cancel_actions', ['/navigate_to_pose'])
        self.declare_parameter('streams', list(DEFAULT_STREAMS))

        streams = []
        for name in self.get_parameter('streams').value:
            topic, type_name, timeout_s = DEFAULT_STREAMS.get(name, ('', '', 1.0))
            self.declare_parameter(f'{name}.topic', topic)
            self.declare_parameter(f'{name}.type', type_name)
            self.declare_parameter(f'{name}.timeout_s', timeout_s)
            topic = self.get_parameter(f'{name}.topic').value
            type_name = self.get_parameter(f'{name}.type').value
            if not topic or not type_name:
                raise ValueError(f'stream {name!r} needs {name}.topic and {name}.type')
            stream = Stream(name, self.get_parameter(f'{name}.timeout_s').value)
            streams.append(stream)
            self.create_subscription(
                get_message(type_name), topic, self._on_stream(name), qos_profile_sensor_data)
            self.get_logger().info(f'stream {name}: {topic} stale after {stream.timeout_s:.2f} s')

        self._policy = WatchdogPolicy(streams, Timing(
            restart_after_s=self.get_parameter('restart_after_s').value,
            startup_grace_s=self.get_parameter('startup_grace_s').value,
            restart_delay_s=self.get_parameter('restart_delay_s').value,
        ))
        self._process = SupervisedProcess(
            self.get_parameter('command').value, self.get_parameter('stop_timeout_s').value)

        self._hold_publisher = self.create_publisher(
            Twist, self.get_parameter('hold_topic').value, 10)
        # Latched, so Foxglove or a later subscriber sees the current state at once.
        self._status_publisher = self.create_publisher(
            String, self.get_parameter('status_topic').value,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self._status = None
        self._cancel_clients = {
            action: self.create_client(CancelGoal, f'{action}/_action/cancel_goal')
            for action in self.get_parameter('cancel_actions').value
        }
        self._timer = self.create_timer(1.0 / self.get_parameter('tick_rate_hz').value, self._on_tick)

    def _on_stream(self, name: str):
        def callback(_msg):
            self._policy.heard(name, time.monotonic())
        return callback

    def _on_tick(self):
        now = time.monotonic()
        returncode = self._process.poll(now)
        if returncode is not None:
            self._policy.exited(now, returncode)

        decision = self._policy.decide(now)
        if decision.cancel_goals:
            self._cancel_goals()
        if decision.stop:
            self._process.request_stop(now)
        if decision.start:
            self._start(now)
        if decision.hold:
            self._hold_publisher.publish(Twist())
        self._report(decision.status)

    def _start(self, now: float):
        command = ' '.join(self._process.command)
        try:
            self._process.start()
        except OSError as e:
            self.get_logger().error(f'could not start {command}: {e}')
            self._policy.started(now)
            self._policy.exited(now, None)
            return
        self._policy.started(now)
        self.get_logger().info(f'started {command} (pid {self._process.pid})')

    def _cancel_goals(self):
        for action, client in self._cancel_clients.items():
            if not client.service_is_ready():
                self.get_logger().info(f'{action} is not available; no Goal to cancel')
                continue
            # A zero goal id and a zero stamp cancel every goal of the action.
            future = client.call_async(CancelGoal.Request())
            future.add_done_callback(self._on_cancel_response(action))

    def _on_cancel_response(self, action: str):
        def callback(future):
            response = future.result()
            if response is not None:
                self.get_logger().warn(
                    f'perception lost: cancelling {len(response.goals_canceling)} {action} goal(s)')
        return callback

    def _report(self, status: str):
        if status == self._status:
            return
        self._status = status
        msg = String()
        msg.data = status
        self._status_publisher.publish(msg)
        if status == 'healthy' or status.startswith('starting'):
            self.get_logger().info(f'perception {status}')
        else:
            self.get_logger().warn(f'perception {status}')

    def shutdown_perception(self):
        if self._process.running:
            self.get_logger().info('stopping the perception bring-up')
            self._process.shutdown(SHUTDOWN_TIMEOUT_S)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # launch stops us with SIGINT; rclpy has already begun shutting down
    finally:
        node.shutdown_perception()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
