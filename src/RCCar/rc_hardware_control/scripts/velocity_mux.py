#!/usr/bin/env python3
"""Velocity mux: one Twist output for the Steering controller, fed by teleop or Nav2.

Teleop outranks Nav2. A source that goes silent for longer than its timeout
drops out, so releasing the teleop panel hands the car back to Nav2 and a dead
Nav2 stops the car. When no source is fresh the mux publishes zero at
publish_rate_hz. Fresh commands pass straight through, so the Steering
controller sees the source's own timing.

Parameters (see config/velocity_mux.yaml for the values and why):
  output_topic, publish_rate_hz, sources (list of names), and per source
  <name>.topic, <name>.priority, <name>.timeout_s.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from rc_hardware_control.velocity_mux_policy import MuxPolicy, Source


class VelocityMux(Node):
    def __init__(self):
        super().__init__('velocity_mux')
        self.declare_parameter('output_topic', '/cmd_vel_mux')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('sources', ['teleop', 'nav'])

        sources = []
        topics = {}
        names = self.get_parameter('sources').value
        for position, name in enumerate(names):
            # Without a yaml: topic /cmd_vel_<name>, earlier in the list wins.
            self.declare_parameter(f'{name}.topic', f'/cmd_vel_{name}')
            self.declare_parameter(f'{name}.priority', len(names) - position)
            self.declare_parameter(f'{name}.timeout_s', 0.5)
            topics[name] = self.get_parameter(f'{name}.topic').value
            sources.append(Source(
                name=name,
                priority=self.get_parameter(f'{name}.priority').value,
                timeout_s=self.get_parameter(f'{name}.timeout_s').value,
            ))
        self._policy = MuxPolicy(sources)

        output_topic = self.get_parameter('output_topic').value
        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self._source_subscriptions = [
            self.create_subscription(Twist, topics[s.name], self._on_command(s.name), 10)
            for s in self._policy.sources
        ]
        rate = self.get_parameter('publish_rate_hz').value
        self._timer = self.create_timer(1.0 / rate, self._on_tick)
        self._active = None   # name of the source last allowed through, or None

        for s in self._policy.sources:
            self.get_logger().info(
                f'source {s.name}: {topics[s.name]} priority {s.priority} timeout {s.timeout_s:.2f} s')
        self.get_logger().info(f'output {output_topic}, zero at {rate:.0f} Hz when no source is fresh')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_command(self, source: str):
        def callback(msg: Twist):
            now = self._now()
            self._policy.offer(source, msg, now)
            selected = self._policy.select(now)
            if selected is not None and selected.source == source:
                self._note_active(source)
                self._publisher.publish(msg)
        return callback

    def _on_tick(self):
        if self._policy.select(self._now()) is None:
            self._note_active(None)
            self._publisher.publish(Twist())

    def _note_active(self, source):
        if source != self._active:
            self.get_logger().info(f'{source or "no fresh source"} is driving')
            self._active = source


def main(args=None):
    rclpy.init(args=args)
    node = VelocityMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # launch stops us with SIGINT; rclpy has already begun shutting down
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
