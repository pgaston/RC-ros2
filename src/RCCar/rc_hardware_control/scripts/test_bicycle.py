#!/usr/bin/env python3

"""
Test script for PCA9685 Bicycle steering control
Demonstrates cmd_vel control using the Bicycle steering controller
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math


class BicycleTestNode(Node):
    def __init__(self):
        super().__init__('bicycle_test')
        
        # rccarauto.launch.py remaps the bicycle steering controller's reference
        # topics onto /cmd_vel, so this is the one topic that drives the car.
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Bicycle Steering Test Node started')
        
        # Timer for testing
        self.timer = self.create_timer(0.1, self.test_callback)
        self.start_time = time.time()
        
    def test_callback(self):
        current_time = time.time() - self.start_time
        
        max_speed = 0.15  # m/s, well under the Nav2 debugging cap
        max_angular = 0.8  # Maximum angular velocity
        
        # Test sequence phases
        if current_time < 3.0:
            # Phase 1: Move forward
            self.send_cmd_vel(max_speed, 0.0)
            self.get_logger().info('Phase 1: Moving forward', throttle_duration_sec=1.0)
            
        elif current_time < 6.0:
            # Phase 2: Turn right while moving forward
            self.send_cmd_vel(max_speed * 0.6, -max_angular * 0.6)
            self.get_logger().info('Phase 2: Turning right', throttle_duration_sec=1.0)
            
        elif current_time < 9.0:
            # Phase 3: Turn left while moving forward
            self.send_cmd_vel(max_speed * 0.6, max_angular * 0.6)
            self.get_logger().info('Phase 3: Turning left', throttle_duration_sec=1.0)
            
        elif current_time < 12.0:
            # Phase 4: Reverse with gentle steering. A bench exercise of the ESC's
            # reverse path; on the road the car reverses only during a Recovery.
            self.send_cmd_vel(-max_speed * 0.4, max_angular * 0.3)
            self.get_logger().info('Phase 4: Moving backward with steering', throttle_duration_sec=1.0)
            
        elif current_time < 15.0:
            # Phase 5: Sharp turn (bicycle-style)
            self.send_cmd_vel(max_speed * 0.3, max_angular)
            self.get_logger().info('Phase 5: Sharp bicycle turn', throttle_duration_sec=1.0)
            
        elif current_time < 17.0:
            # Phase 6: Gradual stop
            decel_factor = max(0.0, (17.0 - current_time) / 2.0)
            self.send_cmd_vel(max_speed * 0.3 * decel_factor, 0.0)
            self.get_logger().info('Phase 6: Gradual stop', throttle_duration_sec=1.0)
            
        else:
            # Phase 7: Complete stop
            self.send_cmd_vel(0.0, 0.0)
            if current_time < 18.0:
                self.get_logger().info('Phase 7: Stopped', throttle_duration_sec=1.0)
        
    def send_cmd_vel(self, linear_x, angular_z):
        """Send a velocity command to the bicycle steering controller via /cmd_vel"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    node = BicycleTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Bicycle test interrupted by user')
    finally:
        # Send stop commands
        try:
            node.send_cmd_vel(0.0, 0.0)
        except Exception as e:
            print(f"Failed to send stop command: {e}")
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()