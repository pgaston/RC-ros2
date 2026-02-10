#!/usr/bin/env python3

"""
Simple script to turn off LED immediately
Use this after launching controllers to ensure LED starts in off state
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time


class LEDOffNode(Node):
    def __init__(self):
        super().__init__('led_off')
        
        # Publisher for LED control (now using effort interface)
        self.led_pub = self.create_publisher(
            Float64MultiArray, 
            '/led_controller/commands', 
            10
        )
        
        # Wait a moment for publisher to be ready
        time.sleep(0.5)
        
        # Send LED off command multiple times to ensure it takes
        for i in range(5):
            led_msg = Float64MultiArray()
            led_msg.data = [0.0]
            self.led_pub.publish(led_msg)
            time.sleep(0.1)
        
        self.get_logger().info('LED turned OFF - sent multiple commands to ensure it takes effect')
        
        # Exit after sending commands
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = LEDOffNode()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()