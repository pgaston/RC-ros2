#!/usr/bin/env python3
"""Tracking recorder: how far the car strays from Nav2's plan on each Goal (issue #7).

For Path follower tuning. Run it next to the full launch:
  ros2 run rc_hardware_control tracking_recorder.py
It follows each Goal from the goal relay's accepted to its final status and
measures the cross-track error (distance from the car to the nearest point of
the path, both in odom) at every odometry message. When a Goal ends it logs a
summary and appends it to goals_<start time>.csv; every sample goes to
samples_<start time>.csv, both in output_dir. The error against the latest plan
is published for a Foxglove plot. #7 passes when the car stays within half a
car width of the plan (chassis_width / 2 from the xacro).

Parameters: plan_topic, odometry_topic, status_topic, error_topic, output_dir,
limit_m (0 means half the chassis width).
"""
import csv
import datetime
import os

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, String

from rc_hardware_control import vehicle_geometry
from rc_hardware_control.tracking_error import GoalSummary, GoalTrack

FINAL_STATUSES = ('arrived', 'stuck', 'aborted', 'rejected')


class TrackingRecorder(Node):
    def __init__(self):
        super().__init__('tracking_recorder')
        plan_topic = self._parameter('plan_topic', '/plan')
        odometry_topic = self._parameter('odometry_topic', '/visual_slam/tracking/odometry')
        status_topic = self._parameter('status_topic', '/goal_relay/status')
        error_topic = self._parameter('error_topic', '/tracking/cross_track_error')
        output_dir = os.path.expanduser(self._parameter('output_dir', '~/.ros/tracking'))
        limit_m = self._parameter('limit_m', 0.0)
        self._limit_m = limit_m if limit_m > 0.0 else vehicle_geometry.load_properties()['chassis_width'] / 2.0

        os.makedirs(output_dir, exist_ok=True)
        started = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        self._goals_path = os.path.join(output_dir, f'goals_{started}.csv')
        with open(self._goals_path, 'w', newline='') as f:
            csv.writer(f).writerow(GoalSummary.header())
        self._samples_file = open(os.path.join(output_dir, f'samples_{started}.csv'), 'w', newline='')
        self._samples = csv.writer(self._samples_file)
        self._samples.writerow(['t', 'goal', 'x', 'y', 'error_m', 'error_first_plan_m'])

        self._goal_count = 0
        self._track = None
        self._plan_frame = None
        self._warned_frames = False
        self._error_publisher = self.create_publisher(Float32, error_topic, 10)
        # Volatile: a status latched from before this node started is not a Goal it saw begin.
        self.create_subscription(String, status_topic, self._on_status, 10)
        self.create_subscription(Path, plan_topic, self._on_plan, 10)
        self.create_subscription(Odometry, odometry_topic, self._on_odometry, qos_profile_sensor_data)
        self.get_logger().info(
            f'recording to {output_dir}; limit {self._limit_m:.3f} m (half a car width)')

    def _parameter(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_status(self, msg: String):
        value = msg.data.split(':', 1)[0]
        now = self._now()
        if value == 'accepted':
            if self._track is not None:
                self._finish(now, 'ended without a final status')
            self._goal_count += 1
            self._track = GoalTrack(self._goal_count, now, self._limit_m)
        elif value in FINAL_STATUSES and self._track is not None:
            self._finish(now, msg.data)

    def _on_plan(self, msg: Path):
        self._plan_frame = msg.header.frame_id
        if self._track is not None:
            self._track.plan([(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    def _on_odometry(self, msg: Odometry):
        if self._track is None:
            return
        if self._plan_frame and msg.header.frame_id != self._plan_frame and not self._warned_frames:
            self._warned_frames = True
            self.get_logger().warn(
                f'odometry is in {msg.header.frame_id} but the plan is in {self._plan_frame}; '
                'errors are meaningless')
        p = msg.pose.pose.position
        errors = self._track.position(p.x, p.y)
        if errors is None:
            return
        self._samples.writerow([f'{self._now():.3f}', self._track.goal, f'{p.x:.3f}', f'{p.y:.3f}',
                                f'{errors[0]:.4f}', f'{errors[1]:.4f}'])
        self._error_publisher.publish(Float32(data=float(errors[0])))

    def _finish(self, now: float, outcome: str):
        summary = self._track.finish(now, outcome)
        self._track = None
        with open(self._goals_path, 'a', newline='') as f:
            csv.writer(f).writerow(summary.row())
        self._samples_file.flush()
        self.get_logger().info(summary.describe())

    def close(self):
        self._samples_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = TrackingRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # launch or Ctrl-C stops us with SIGINT; rclpy has already begun shutting down
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
