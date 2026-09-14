#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
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
    try:
        rclpy.spin(cmd_vel_logger)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # launch stops us with SIGINT; rclpy has already begun shutting down
    finally:
        cmd_vel_logger.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
