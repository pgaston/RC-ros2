#!/usr/bin/env python3
"""
Stereo Sync Republisher - Synchronizes stereo pairs with ExactTime matching.

RealSense with enable_sync=true gives matching timestamps, but the ROS driver
publishes infra1 and infra2 at slightly different times. This causes cuVSLAM
to grab mismatched pairs. This node uses ExactTimeSynchronizer to guarantee
paired frames have identical timestamps.

Also fixes the RealSense driver bug where infra2 has wrong frame_id.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, TimeSynchronizer

class StereoSyncRepublisher(Node):
    def __init__(self):
        super().__init__('stereo_sync_republisher')
        
        # QoS matching camera's SENSOR_DATA profile
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers using message_filters for EXACT time sync
        self.img1_sub = Subscriber(self, Image, '/camera/infra1/image_rect_raw', qos_profile=sensor_qos)
        self.img2_sub = Subscriber(self, Image, '/camera/infra2/image_rect_raw', qos_profile=sensor_qos)
        self.info1_sub = Subscriber(self, CameraInfo, '/camera/infra1/camera_info', qos_profile=sensor_qos)
        self.info2_sub = Subscriber(self, CameraInfo, '/camera/infra2/camera_info', qos_profile=sensor_qos)
        
        # ExactTimeSynchronizer - RealSense enable_sync=true guarantees matching timestamps
        self.sync = TimeSynchronizer(
            [self.img1_sub, self.img2_sub, self.info1_sub, self.info2_sub],
            queue_size=30  # Buffer to handle publish timing differences
        )
        self.sync.registerCallback(self.sync_callback)
        
        # Publishers to synced topics
        self.img1_pub = self.create_publisher(Image, '/synced/infra1/image_rect_raw', sensor_qos)
        self.img2_pub = self.create_publisher(Image, '/synced/infra2/image_rect_raw', sensor_qos)
        self.info1_pub = self.create_publisher(CameraInfo, '/synced/infra1/camera_info', sensor_qos)
        self.info2_pub = self.create_publisher(CameraInfo, '/synced/infra2/camera_info', sensor_qos)
        
        self.count = 0
        self.get_logger().info('Stereo Sync Republisher started (ExactTime sync)')
        
    def sync_callback(self, img1, img2, info1, info2):
        self.count += 1
        
        # FIX: RealSense driver bug - infra2 publishes wrong frame_id
        img2.header.frame_id = 'camera_infra2_optical_frame'
        info2.header.frame_id = 'camera_infra2_optical_frame'
        
        # Publish synchronized pairs
        self.img1_pub.publish(img1)
        self.img2_pub.publish(img2)
        self.info1_pub.publish(info1)
        self.info2_pub.publish(info2)
        
        if self.count % 30 == 0:
            self.get_logger().info(f'Synced {self.count} stereo pairs')

def main():
    rclpy.init()
    node = StereoSyncRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
