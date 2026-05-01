#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose

import tf2_ros
import tf2_geometry_msgs


class GoalPoseRelay(Node):
    def __init__(self):
        super().__init__('goal_pose_relay')

        self.declare_parameter('default_goal_frame', 'odom')
        self.declare_parameter('action_name', 'navigate_to_pose')

        self.default_goal_frame = self.get_parameter('default_goal_frame').value
        action_name = self.get_parameter('action_name').value

        self.nav_action_client = ActionClient(self, NavigateToPose, action_name)
        self.pending_goal_handle = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)

    def goal_pose_callback(self, msg):
        self.send_goal(msg.pose.position.x, msg.pose.position.y, msg.header.frame_id)

    def clicked_point_callback(self, msg):
        self.send_goal(msg.point.x, msg.point.y, msg.header.frame_id)

    def send_goal(self, x, y, frame_id):
        if not self.nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('navigate_to_pose action server is not available')
            return

        target_frame = self.default_goal_frame
        pose_in = PoseStamped()
        pose_in.header.frame_id = frame_id or target_frame
        pose_in.header.stamp = Time().to_msg() 
        pose_in.pose.position.x = x
        pose_in.pose.position.y = y
        pose_in.pose.orientation.w = 1.0

        if pose_in.header.frame_id != target_frame:
            try:
                pose_out = self.tf_buffer.transform(pose_in, target_frame, timeout=Duration(seconds=1.0))
            except Exception as e:
                self.get_logger().error(f'Failed to transform goal pose from {pose_in.header.frame_id} to {target_frame}: {e}')
                return
        else:
            pose_out = pose_in

        goal = NavigateToPose.Goal()
        goal.pose = pose_out
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f'Sending Nav2 goal to x={goal.pose.pose.position.x:.2f}, y={goal.pose.pose.position.y:.2f} in frame {goal.pose.header.frame_id}')

        send_goal_future = self.nav_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the relayed goal')
            return

        self.pending_goal_handle = goal_handle
        self.get_logger().info('Nav2 accepted the relayed goal')

    
def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()