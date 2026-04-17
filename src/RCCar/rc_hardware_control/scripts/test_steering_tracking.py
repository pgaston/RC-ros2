#!/usr/bin/env python3

"""
Test script for PCA9685 steering and tracking joints
Demonstrates position control for steering servo and camera pan/tilt,
and velocity control for traction motor.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import math


class SteeringTrackingTestNode(Node):
    def __init__(self):
        super().__init__('steering_tracking_test')
        
        # Publishers for different controllers
        self.steering_pub = self.create_publisher(
            Float64MultiArray, 
            '/steering_controller/commands', 
            10
        )
        
        self.tracking_pub = self.create_publisher(
            Float64MultiArray, 
            '/tracking_controller/commands', 
            10
        )
        
        self.get_logger().info('Steering and Tracking Test Node started')
        
        # Timer for testing
        self.timer = self.create_timer(0.1, self.test_callback)
        self.test_phase = 0
        self.start_time = time.time()
        
    def test_callback(self):
        current_time = time.time() - self.start_time
        
        # Test steering (position control) - sweep left and right
        steering_angle = 0.5 * math.sin(current_time * 0.5)  # ±0.5 radians
        steering_msg = Float64MultiArray()
        steering_msg.data = [steering_angle]
        self.steering_pub.publish(steering_msg)
        
        # Test tracking (velocity control for traction only)
        traction_velocity = 0.1 if current_time < 10.0 else 0.0
        tracking_msg = Float64MultiArray()
        tracking_msg.data = [traction_velocity]
        self.tracking_pub.publish(tracking_msg)
        
        # # Test pan/tilt (position control)
        # pan_angle = 0.8 * math.sin(current_time * 0.3)
        # tilt_angle = 0.3 * math.sin(current_time * 0.4)
        # pan_tilt_msg = Float64MultiArray()
        # pan_tilt_msg.data = [pan_angle, tilt_angle]
        # self.pan_tilt_pub.publish(pan_tilt_msg)
        
        # Log current commands
        if int(current_time) % 2 == 0 and current_time - int(current_time) < 0.1:
            self.get_logger().info(
                f'Steering: {steering_angle:.2f} rad, '
                f'Traction: {traction_velocity:.2f} m/s, '
                # f'Pan: {pan_angle:.2f} rad, Tilt: {tilt_angle:.2f} rad'
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = SteeringTrackingTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted by user')
    finally:
        # Send stop commands
        steering_msg = Float64MultiArray()
        steering_msg.data = [0.0]
        node.steering_pub.publish(steering_msg)
        
        tracking_msg = Float64MultiArray()
        tracking_msg.data = [0.0]  # Stop traction motor
        node.tracking_pub.publish(tracking_msg)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()