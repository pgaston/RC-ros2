#!/usr/bin/env python3

"""
Test script for PCA9685 Ackermann steering control
Demonstrates cmd_vel control using the Ackermann steering controller
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import time
import math


class AckermannTestNode(Node):
    def __init__(self):
        super().__init__('ackermann_test')
        
        # Publisher for Ackermann commands
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/ackermann_steering_controller/cmd_vel', 
            10
        )
        
        # Publishers for LED control
        self.led_pub = self.create_publisher(
            Float64MultiArray, 
            '/led_controller/commands', 
            10
        )
        
        self.get_logger().info('Ackermann Test Node started')
        
        # Timer for testing
        self.timer = self.create_timer(0.1, self.test_callback)
        self.start_time = time.time()
        
    def test_callback(self):
        current_time = time.time() - self.start_time
        
        max_speed = 0.1
        # Test sequence phases
        if current_time < 3.0:
            # Phase 1: Move forward
            self.send_cmd_vel(max_speed, 0.0)
            self.get_logger().info('Phase 1: Moving forward', throttle_duration_sec=1.0)
            
        elif current_time < 6.0:
            # Phase 2: Turn right while moving forward
            self.send_cmd_vel(max_speed/2, -0.5)
            self.get_logger().info('Phase 2: Turning right', throttle_duration_sec=1.0)
            
        elif current_time < 9.0:
            # Phase 3: Turn left while moving forward
            self.send_cmd_vel(0.max_speed, 0.5)
            self.get_logger().info('Phase 3: Turning left', throttle_duration_sec=1.0)
            
        elif current_time < 12.0:
            # Phase 4: Reverse
            self.send_cmd_vel(-max_speed0.3, 0.0)
            self.get_logger().info('Phase 4: Moving backward', throttle_duration_sec=1.0)
            
        elif current_time < 15.0:
            # Phase 5: Spin in place
            self.send_cmd_vel(0.0, 1.0)
            self.get_logger().info('Phase 5: Spinning in place', throttle_duration_sec=1.0)
            
        else:
            # Phase 6: Stop
            self.send_cmd_vel(0.0, 0.0)
            if current_time < 16.0:
                self.get_logger().info('Phase 6: Stopped', throttle_duration_sec=1.0)
        
        # Control LED brightness based on speed
        speed = abs(self.last_linear) + abs(self.last_angular)
        led_brightness = min(1.0, speed / 1.0)  # Scale to 0-1
        led_msg = Float64MultiArray()
        led_msg.data = [led_brightness]
        self.led_pub.publish(led_msg)
        
    def send_cmd_vel(self, linear_x, angular_z):
        """Send velocity command to Ackermann controller"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)
        
        # Store last command for other controls
        self.last_linear = linear_x
        self.last_angular = angular_z

    def __init_vars(self):
        self.last_linear = 0.0
        self.last_angular = 0.0

# Initialize vars after __init__
AckermannTestNode.last_linear = 0.0
AckermannTestNode.last_angular = 0.0


def main(args=None):
    rclpy.init(args=args)
    
    node = AckermannTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted by user')
    finally:
        # Send stop commands
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        node.cmd_vel_pub.publish(stop_cmd)
        
        # Turn off LED
        led_msg = Float64MultiArray()
        led_msg.data = [0.0]
        node.led_pub.publish(led_msg)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()