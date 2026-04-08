#!/usr/bin/env python3

# Launch nvblox node for dense 3D mapping after visual_slam_node
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nvblox_node = Node(
        package='isaac_ros_nvblox',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[{
            'use_depth': True,
            'use_color': False,
            'global_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/aligned_depth_to_color/camera_info',
            'pose_topic': '/visual_slam/odom',
        }],
        remappings=[
            ('/nvblox_node/odom', '/visual_slam/odom'),
            ('/nvblox_node/depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('/nvblox_node/depth/camera_info', '/camera/aligned_depth_to_color/camera_info'),
        ]
    )
    return LaunchDescription([
        nvblox_node
    ])
