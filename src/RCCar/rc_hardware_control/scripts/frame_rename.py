#!/usr/bin/env python3

# Frame Rename Node - Fixes CameraInfo frame_id for infra2 camera
# for whatever reason, the second camera_info uses the frame_id from the first camera
# This node subscribes to the original camera_info topic, modifies the frame_id,
# and republishes it to a new topic that VSLAM can use.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo


class CameraInfoFixer(Node):
    def __init__(self):
        super().__init__('vslam_info_fixer')

        # QoS matching camera's SENSOR_DATA profile
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )

        self.info2_sub = self.create_subscription(
            CameraInfo, 
            '/infra2/camera_info', 
            self.callback, sensor_qos)       
        self.info2_pub = self.create_publisher(CameraInfo, '/infra2/camera_info_fixed', sensor_qos)

    def callback(self, msg):
        # Update only the frame_id to the one Isaac ROS VSLAM expects
        msg.header.frame_id = 'camera_infra2_optical_frame'
        self.info2_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
