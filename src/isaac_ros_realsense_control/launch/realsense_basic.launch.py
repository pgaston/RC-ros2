#!/usr/bin/env python3

# Copyright (c) 2025, Your Name. All rights reserved.

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RealSense D435i - system package version."""
    
    # this way, we can change parameters in the visual_slam launch file
    depth_profile_arg = DeclareLaunchArgument('depth_profile', default_value='424x240x30')
    color_profile_arg = DeclareLaunchArgument('color_profile', default_value='424x240x30')
    enable_depth_arg = DeclareLaunchArgument('enable_depth', default_value='false', description='Enable depth stream')
    enable_sync_arg = DeclareLaunchArgument('enable_sync', default_value='false', description='Enable emitter sync')
    emitter_enabled_arg = DeclareLaunchArgument('emitter_enabled', default_value='0', description='Emitter status: 0=disable, 1=enable')
    enable_accel_arg = DeclareLaunchArgument('enable_accel', default_value='false', description='Enable accel')
    enable_gyro_arg = DeclareLaunchArgument('enable_gyro', default_value='false', description='Enable gyro')
    unite_imu_method_arg = DeclareLaunchArgument('unite_imu_method', default_value='0', description='Unite IMU method')

    # Launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera name/namespace'
    )
    
    device_type_arg = DeclareLaunchArgument(
        'device_type',
        default_value='D435i',
        description='RealSense device type'
    )
    
    enable_pointcloud_arg = DeclareLaunchArgument(
        'enable_pointcloud',
        default_value='true',
        description='Enable point cloud generation'
    )


    # Include the standard RealSense launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'device_type': LaunchConfiguration('device_type'),

            # 2. Profiles (Modern D400 Parameter Naming)
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
            'depth_module.infra_profile': LaunchConfiguration('depth_profile'),
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
            'depth_module.emitter_enabled': LaunchConfiguration('emitter_enabled'),

            # 'enable_pointcloud': LaunchConfiguration('enable_pointcloud'),
            'pointcloud.enable': LaunchConfiguration('enable_pointcloud'),
            'enable_sync': LaunchConfiguration('enable_sync'),
            'align_depth.enable': 'true',
            'colorizer.enable': 'true',
            'decimation_filter.enable': 'false',
            'spatial_filter.enable': 'false',
            'temporal_filter.enable': 'false',
            'hole_filling_filter.enable': 'false',
            'enable_color': 'true',
            'enable_depth': LaunchConfiguration('enable_depth'),
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'enable_accel': LaunchConfiguration('enable_accel'),
            'enable_gyro': LaunchConfiguration('enable_gyro'),
            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
            'publish_tf': 'true'
        }.items()
    )

    # RViz node for visualization
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('isaac_ros_realsense_control'),
        'config',
        'realsense_basic.rviz'
    ])
    
    return LaunchDescription([
        depth_profile_arg,
        color_profile_arg,
        emitter_enabled_arg,
        camera_name_arg,
        device_type_arg,
        enable_pointcloud_arg,  # pointcloud.enable (set to true/false), not enable_pointcloud
        enable_accel_arg,
        enable_gyro_arg,
        unite_imu_method_arg,
        enable_sync_arg,
        enable_depth_arg,
        realsense_launch,
    ])