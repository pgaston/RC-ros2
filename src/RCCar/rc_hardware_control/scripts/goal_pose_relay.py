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

Stop: any message on /goal_relay/cancel (std_msgs/Empty; a Foxglove Publish
panel makes it a button) cancels every navigate_to_pose Goal, the relay's and
any other client's, and reports the relay's current Goal aborted. Holding
teleop at zero only pauses a Goal; this ends it.

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
  aborted   ended any other way: preempted by a new Goal, cancelled by the
            operator, perception lost, cancelled outside the relay, or
            aborted by Nav2
Each Goal gets either rejected alone, or accepted followed by one of arrived,
stuck, aborted or rejected. A caller that publishes a Goal takes the messages
that follow as that Goal's answers.

The relay idles on a busy graph: /tf arrives at about 85 Hz on the car and is
kept undecoded, and a transform buffer is built from it only when a Goal
arrives (TfOnDemand). The occupancy grid and the plan are only noted as
received, never decoded.

Parameters: global_frame, robot_frame, action_name, status_topic,
occupancy_grid_topic, perception_status_topic ('' to admit without the
perception watchdog), plan_topic, pose_topic, point_topic, cancel_topic.
"""
import math
import time
from collections import deque

import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from action_msgs.srv import CancelGoal
from std_msgs.msg import Empty, String
from tf2_geometry_msgs import do_transform_point
from tf2_msgs.msg import TFMessage

from rc_hardware_control.goal_relay_policy import (
    Readiness, goal_heading, outcome, quaternion_of_yaw, refusal, yaw_of)

# How long a Goal request waits for Nav2's action server to be discovered.
SERVER_WAIT_S = 0.5

# How much /tf a Goal request can see. Every dynamic transform on the car is
# published at 30 Hz; one published less often than this would be missed, and a
# car pose older than this (visual SLAM stopped) counts as no recent transform.
TF_KEEP_S = 2.0


class TfOnDemand:
    """/tf and /tf_static kept as received, decoded into a tf2 Buffer only when asked.

    A TransformListener decodes every /tf message in Python, which cost the
    relay a quarter of a core on the car (about 85 messages a second) although
    it needs a transform only when a Goal arrives.
    """

    def __init__(self, node: Node):
        self._static = []       # every /tf_static message, undecoded
        self._recent = deque()  # (arrival, undecoded /tf message) for the last TF_KEEP_S
        node.create_subscription(TFMessage, '/tf', self._on_tf, QoSProfile(depth=100), raw=True)
        node.create_subscription(
            TFMessage, '/tf_static', self._static.append,
            QoSProfile(depth=100, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL), raw=True)

    def _on_tf(self, raw: bytes):
        now = time.monotonic()
        self._recent.append((now, raw))
        self._forget_before(now - TF_KEEP_S)

    def _forget_before(self, cutoff: float):
        while self._recent and self._recent[0][0] < cutoff:
            self._recent.popleft()

    def buffer(self) -> tf2_ros.Buffer:
        # Also here: if /tf stopped altogether, nothing else would drop the old messages.
        self._forget_before(time.monotonic() - TF_KEEP_S)
        buffer = tf2_ros.Buffer()
        for raw in self._static:
            for transform in deserialize_message(raw, TFMessage).transforms:
                buffer.set_transform_static(transform, 'goal_relay')
        for _, raw in self._recent:
            for transform in deserialize_message(raw, TFMessage).transforms:
                buffer.set_transform(transform, 'goal_relay')
        return buffer


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
        cancel_topic = self._parameter('cancel_topic', '/goal_relay/cancel')

        self._client = ActionClient(self, NavigateToPose, self._action)
        self._cancel_all = self.create_client(CancelGoal, f'{self._action}/_action/cancel_goal')
        self._tf = TfOnDemand(self)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status_publisher = self.create_publisher(String, status_topic, latched)

        self._grid_received = False
        self.create_subscription(
            OccupancyGrid, grid_topic, self._on_grid, qos_profile_sensor_data, raw=True)
        self._perception_watched = bool(perception_topic)
        self._perception_status = None
        if self._perception_watched:
            self.create_subscription(String, perception_topic, self._on_perception_status, latched)
        self._last_plan_at = None
        self.create_subscription(Path, plan_topic, self._on_plan, 10, raw=True)

        self._current = None
        self.create_subscription(
            PoseStamped, pose_topic, lambda m: self._on_request(m.header.frame_id, m.pose.position), 10)
        self.create_subscription(
            PointStamped, point_topic, lambda m: self._on_request(m.header.frame_id, m.point), 10)
        self.create_subscription(Empty, cancel_topic, self._on_cancel, 10)
        self.get_logger().info(
            f'Goals from {pose_topic} and {point_topic} to {self._action} in {self._global_frame}; '
            f'stop on {cancel_topic}; status on {status_topic}')

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
        tf = self._tf.buffer()
        reason = refusal(Readiness(
            server_available=server_available,
            robot_pose_known=tf.can_transform(self._global_frame, self._robot_frame, Time()),
            grid_received=self._grid_received,
            perception_watched=self._perception_watched,
            perception_status=self._perception_status,
        ), self._action, self._global_frame, self._robot_frame)
        if reason is not None:
            self._report('rejected', reason)
            return

        try:
            goal_x, goal_y = self._in_global_frame(tf, frame, point)
            robot = tf.lookup_transform(self._global_frame, self._robot_frame, Time())
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

    def _in_global_frame(self, tf: tf2_ros.Buffer, frame: str, point):
        if frame == self._global_frame:
            return point.x, point.y
        transform = tf.lookup_transform(self._global_frame, frame, Time())
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

    def _on_cancel(self, _msg):
        current = self._current
        if current is not None and not current.finished:
            current.finished = True
            self._report('aborted', 'cancelled by the operator')
            # A Goal Nav2 has not answered yet is cancelled when the answer comes.
        else:
            self.get_logger().info('stop requested with no Goal of ours running; cancelling any other')
        # An all-zero goal id and stamp cancels every Goal on the server, so a
        # Goal sent around the relay (ros2 action send_goal) stops too. This
        # also cancels the relay's own, so its handle needs no separate cancel.
        if self._cancel_all.service_is_ready():
            self._cancel_all.call_async(CancelGoal.Request())
        else:
            self.get_logger().error(f'cannot stop: {self._action} cancel service is not available')

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
