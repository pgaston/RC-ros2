#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelLogger(Node):
    def __init__(self):
        super().__init__('cmd_vel_logger')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.get_logger().info('CmdVelLogger initialized subscribing to /cmd_vel (Twist)')

    def listener_callback(self, msg):
        x = msg.linear.x
        z = msg.angular.z
        self.get_logger().info(f'CMD_VEL RECEIVED: x={x:.4f}, z={z:.4f}')


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_logger = CmdVelLogger()
    rclpy.spin(cmd_vel_logger)
    cmd_vel_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
