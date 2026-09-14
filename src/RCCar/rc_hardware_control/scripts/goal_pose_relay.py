#!/usr/bin/env python3
"""Goal relay: the only NavigateToPose client. Admits a Goal, sends it to Nav2, reports how it ended.

A point clicked in Foxglove (/clicked_point, PointStamped) or a pose
(/goal_pose, PoseStamped) asks for a Goal; both behave the same, and only the
position is used. The relay admits it only when Nav2's action server is up,
the transform from the global frame (odom) to the robot frame exists, an
occupancy grid has arrived, and the perception watchdog reports healthy. An
admitted Goal is transformed into odom, faces along the bearing from the car
(goal_relay_policy.goal_heading), and goes to navigate_to_pose. A new Goal
replaces the current one: the current one is reported aborted and cancelled,
then the new one is sent.

Status topic, for Foxglove and for any program that sends the car somewhere
(the future VLM brain):
  /goal_relay/status  std_msgs/String, reliable, transient local with depth 1
                      (a late subscriber gets the latest), one message per
                      change, '<value>: <reason>'; split on the first ': '.
  accepted  Nav2 took the Goal; the reason gives its position and heading in odom
  rejected  refused without driving: the relay is not ready (the reason names
            the missing fact), the Goal could not be transformed into odom,
            Nav2 refused it, or Nav2 found no path to it (a Goal in a wall)
  arrived   within the Arrival tolerance
  stuck     Nav2 gave up after at least one Recovery
  aborted   ended any other way: preempted by a new Goal, perception lost,
            cancelled outside the relay, or aborted by Nav2
Each Goal gets either rejected alone, or accepted followed by one of arrived,
stuck, aborted or rejected. A caller that publishes a Goal takes the messages
that follow as that Goal's answers.

Parameters: global_frame, robot_frame, action_name, status_topic,
occupancy_grid_topic, perception_status_topic ('' to admit without the
perception watchdog), plan_topic, pose_topic, point_topic.
"""
import math
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point

from rc_hardware_control.goal_relay_policy import (
    Readiness, goal_heading, outcome, quaternion_of_yaw, refusal, yaw_of)

# How long a Goal request waits for Nav2's action server to be discovered.
SERVER_WAIT_S = 0.5


class SentGoal:
    """A Goal sent to Nav2, from the send until a final status is published for it."""

    def __init__(self, x: float, y: float, heading: float, sent_at: float):
        self.x, self.y, self.heading, self.sent_at = x, y, heading, sent_at
        self.handle = None      # set when Nav2 accepts it
        self.recoveries = 0     # Nav2's number_of_recoveries feedback, highest seen
        self.finished = False   # a final status (or the preemption) has been published


class GoalRelay(Node):
    def __init__(self):
        super().__init__('goal_pose_relay')
        self._global_frame = self._parameter('global_frame', 'odom')
        self._robot_frame = self._parameter('robot_frame', 'base_footprint')
        self._action = self._parameter('action_name', 'navigate_to_pose')
        status_topic = self._parameter('status_topic', '/goal_relay/status')
        grid_topic = self._parameter('occupancy_grid_topic', '/nvblox_node/static_occupancy_grid')
        perception_topic = self._parameter('perception_status_topic', '/perception/status')
        plan_topic = self._parameter('plan_topic', '/plan')
        pose_topic = self._parameter('pose_topic', '/goal_pose')
        point_topic = self._parameter('point_topic', '/clicked_point')

        self._client = ActionClient(self, NavigateToPose, self._action)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status_publisher = self.create_publisher(String, status_topic, latched)

        self._grid_received = False
        self.create_subscription(OccupancyGrid, grid_topic, self._on_grid, qos_profile_sensor_data)
        self._perception_watched = bool(perception_topic)
        self._perception_status = None
        if self._perception_watched:
            self.create_subscription(String, perception_topic, self._on_perception_status, latched)
        self._last_plan_at = None
        self.create_subscription(Path, plan_topic, self._on_plan, 10)

        self._current = None
        self.create_subscription(
            PoseStamped, pose_topic, lambda m: self._on_request(m.header.frame_id, m.pose.position), 10)
        self.create_subscription(
            PointStamped, point_topic, lambda m: self._on_request(m.header.frame_id, m.point), 10)
        self.get_logger().info(
            f'Goals from {pose_topic} and {point_topic} to {self._action} in {self._global_frame}; '
            f'status on {status_topic}')

    def _parameter(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _on_grid(self, _msg):
        self._grid_received = True

    def _on_perception_status(self, msg):
        self._perception_status = msg.data

    def _on_plan(self, _msg):
        self._last_plan_at = time.monotonic()

    def _on_request(self, frame_id: str, point):
        frame = frame_id or self._global_frame
        server_available = (self._client.server_is_ready()
                            or self._client.wait_for_server(timeout_sec=SERVER_WAIT_S))
        reason = refusal(Readiness(
            server_available=server_available,
            robot_pose_known=self._tf_buffer.can_transform(self._global_frame, self._robot_frame, Time()),
            grid_received=self._grid_received,
            perception_watched=self._perception_watched,
            perception_status=self._perception_status,
        ), self._action, self._global_frame, self._robot_frame)
        if reason is not None:
            self._report('rejected', reason)
            return

        try:
            goal_x, goal_y = self._in_global_frame(frame, point)
            robot = self._tf_buffer.lookup_transform(self._global_frame, self._robot_frame, Time())
        except tf2_ros.TransformException as e:
            self._report('rejected', f'cannot transform the Goal from {frame} to {self._global_frame}: {e}')
            return
        r = robot.transform
        heading = goal_heading(r.translation.x, r.translation.y,
                               yaw_of(r.rotation.x, r.rotation.y, r.rotation.z, r.rotation.w),
                               goal_x, goal_y)

        request = NavigateToPose.Goal()
        request.pose.header.frame_id = self._global_frame
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x = goal_x
        request.pose.pose.position.y = goal_y
        q = request.pose.pose.orientation
        q.x, q.y, q.z, q.w = quaternion_of_yaw(heading)

        self._preempt_current()
        sent = SentGoal(goal_x, goal_y, heading, time.monotonic())
        self._current = sent
        future = self._client.send_goal_async(request, feedback_callback=self._on_feedback(sent))
        future.add_done_callback(self._on_response(sent))

    def _in_global_frame(self, frame: str, point):
        if frame == self._global_frame:
            return point.x, point.y
        transform = self._tf_buffer.lookup_transform(self._global_frame, frame, Time())
        stamped = PointStamped()
        stamped.header.frame_id = frame
        stamped.point = point
        p = do_transform_point(stamped, transform).point
        return p.x, p.y

    def _preempt_current(self):
        current = self._current
        if current is None or current.finished:
            return
        current.finished = True
        self._report('aborted', 'preempted by a new Goal')
        if current.handle is not None:
            current.handle.cancel_goal_async()
        # A Goal Nav2 has not answered yet is cancelled when the answer comes.

    def _on_feedback(self, sent: SentGoal):
        def callback(msg):
            sent.recoveries = max(sent.recoveries, msg.feedback.number_of_recoveries)
        return callback

    def _on_response(self, sent: SentGoal):
        def callback(future):
            handle = future.result()
            if sent.finished:   # replaced before Nav2 answered
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                sent.finished = True
                self._report('rejected', f'{self._action} refused the Goal')
                return
            sent.handle = handle
            self._report('accepted', f'going to ({sent.x:.2f}, {sent.y:.2f}) in {self._global_frame}, '
                                     f'heading {math.degrees(sent.heading):.0f} deg')
            handle.get_result_async().add_done_callback(self._on_result(sent))
        return callback

    def _on_result(self, sent: SentGoal):
        def callback(future):
            if sent.finished:
                return
            sent.finished = True
            plan_received = self._last_plan_at is not None and self._last_plan_at >= sent.sent_at
            perception = self._perception_status if self._perception_watched else None
            value, reason = outcome(future.result().status, sent.recoveries, plan_received, perception)
            self._report(value, reason)
        return callback

    def _report(self, value: str, reason: str):
        msg = String()
        msg.data = f'{value}: {reason}'
        self._status_publisher.publish(msg)
        if value in ('accepted', 'arrived'):
            self.get_logger().info(msg.data)
        else:
            self.get_logger().warn(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = GoalRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # launch stops us with SIGINT; rclpy has already begun shutting down
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
