#!/usr/bin/env python3

"""
Test script for PCA9685 Bicycle steering control
Demonstrates cmd_vel control using the Bicycle steering controller
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import time
import math


class BicycleTestNode(Node):
    def __init__(self):
        super().__init__('bicycle_test')
        
        # Initialize instance variables
        self.last_linear = 0.0
        self.last_angular = 0.0
        
        # Publisher for Bicycle steering commands (using reference_unstamped - only available topic)
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/bicycle_steering_controller/reference_unstamped', 
            10
        )
        
        # Publishers for LED control
        self.led_pub = self.create_publisher(
            Float64MultiArray, 
            '/led_controller/commands', 
            10
        )
        
        self.get_logger().info('Bicycle Steering Test Node started')
        
        # Turn off LED immediately at startup
        self.turn_off_led()
        
        # Timer for testing
        self.timer = self.create_timer(0.1, self.test_callback)
        self.start_time = time.time()
        
    def test_callback(self):
        current_time = time.time() - self.start_time
        
        max_speed = 0.15  # Slightly higher for bicycle model
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
            # Phase 4: Reverse with gentle steering
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
        
        # Control LED brightness based on total movement
        speed = abs(self.last_linear) + abs(self.last_angular) * 0.5
        led_brightness = min(1.0, speed / max_speed)  # Scale to 0-1
        led_msg = Float64MultiArray()
        led_msg.data = [led_brightness]
        self.led_pub.publish(led_msg)
        
    def send_cmd_vel(self, linear_x, angular_z):
        """Send velocity command to Bicycle steering controller using reference_unstamped"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)
        
        # Store last command for other controls
        self.last_linear = linear_x
        self.last_angular = angular_z
        
    def turn_off_led(self):
        """Turn off LED immediately"""
        led_msg = Float64MultiArray()
        led_msg.data = [0.0]
        self.led_pub.publish(led_msg)
        self.get_logger().info('LED turned off')


def main(args=None):
    rclpy.init(args=args)
    
    node = BicycleTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Bicycle test interrupted by user')
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