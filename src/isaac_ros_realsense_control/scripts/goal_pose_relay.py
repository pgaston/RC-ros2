#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose


class GoalPoseRelay(Node):
    def __init__(self):
        super().__init__('goal_pose_relay')

        self.declare_parameter('default_goal_frame', 'odom')
        self.declare_parameter('action_name', 'navigate_to_pose')

        self.default_goal_frame = self.get_parameter('default_goal_frame').value
        action_name = self.get_parameter('action_name').value

        self.nav_action_client = ActionClient(self, NavigateToPose, action_name)
        self.pending_goal_handle = None

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

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id or self.default_goal_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Sending Nav2 goal to x={x:.2f}, y={y:.2f} in frame {goal.pose.header.frame_id}')

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