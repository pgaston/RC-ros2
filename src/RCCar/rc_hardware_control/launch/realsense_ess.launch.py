#!/usr/bin/env python3

# Copyright (c) 2025, Your Name. All rights reserved.

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RealSense D435i with Isaac ROS ESS."""

    # Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='camera',
        description='Namespace for camera nodes'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    model_file_path_arg = DeclareLaunchArgument(
        'model_file_path',
        default_value='/tmp/models/bi3d_proximity_segmentation/bi3d_proximity_segmentation.plan',
        description='Path to the ESS model file'
    )

    engine_file_path_arg = DeclareLaunchArgument(
        'engine_file_path',
        # default_value='isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/light_ess.engine',
        default_value='isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/ess.engine',
        description='Path to the TensorRT engine file'
    )

    # RealSense camera node
    realsense_camera_node = Node(
        name='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_color': False,
            'enable_depth': False,
            'infra_width': 848,
            'infra_height': 480,
            'infra_fps': 30,
        }],
    )

    # ESS Disparity Node
    disparity_node = ComposableNode(
        name='disparity',
        package='isaac_ros_ess',
        plugin='nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode',
        parameters=[{
            'engine_file_path': LaunchConfiguration('engine_file_path'),
            'threshold': 0.4,
        }],
        remappings=[
            ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
            ('left/camera_info', '/camera/camera/infra1/camera_info'),
            ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
            ('right/camera_info', '/camera/camera/infra2/camera_info')
        ]
    )

    # Only ESS node for minimal test

    # ESS container
    ess_container = ComposableNodeContainer(
        name='ess_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            disparity_node
        ],
        output='screen'
    )


    # RViz node for visualization
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('rc_hardware_control'),
            'config',
            'realsense_ess.rviz'
        ])],
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        enable_rviz_arg,
        engine_file_path_arg,
        realsense_camera_node,
        ess_container
    ])