#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate stabilized launch for D435i on Orin Nano."""
    
    # 2. Consolidated Profiles (640x480 @ 30fps for stability)
    # Using 640x480 reduces USB bandwidth and decompression CPU cycles.
    # various resolutions:   848x480x30, 640x480x30, 424x240x30, 320x240x30
    # depth not used
    color_profile = '640x480x15'    # slower
    default_profile = '640x480x30'

    # Composable node will have zero copy of images
    realsense_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        namespace='',
        name='camera',
        
        parameters=[{
            'camera_name': 'camera',
            'global_time_enabled': False, 
            'host_performance_step': 'true',

            # 1. TF Management - Set to False because we use our own URDF
            'publish_tf': False, 
            'tf_publish_rate': 30.0, # Force dynamic TFs for the IMU/Optical frames
            'camera_base_frame': 'camera_link',
            
            # 2. Profiles (Modern D400 Parameter Naming)
            # Must set BOTH depth_module.profile AND depth_module.infra_profile
            # librealsense2 does not set default profiles for infra streams!
            'depth_module.profile': default_profile,
            'depth_module.infra_profile': default_profile,
            'rgb_camera.profile': color_profile,

            # 3. Enable Streams for VSLAM & NVBLOX
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_depth': False, # Disable depth to ensure VSLAM gets stable stereo frames
            'enable_color': True, # Save USB bandwidth (VSLAM/Nvblox don't use it in this config)
            # Watch this - as color goes at 15, and infra at 30 - infra may get held up...
            'enable_sync': True,  # Disable strict sync to prevent frame drops causing "sleep"
            
            # 4. IMU Configuration (Crucial for cuVSLAM)
            'gyro_fps': 200,
            'accel_fps': 250,  # D435i supports 63 or 250Hz - use 250 for VIO
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2, # 2 = Linear Interpolation (Standard for VIO)
            
            # 5. Performance & Stability
            'initial_reset': False,        # Enable reset to clear IMU calibration errors
            'reconnect_timeout': 6.0,      # Wait seconds before trying to reconnect
            'wait_for_device_timeout': 30.0, # Wait for device to become available
            'depth_module.emitter_enabled': 0, # Set to 0 if outdoors
            'depth_module.depth_qos': 'SENSOR_DATA',
            'depth_module.exposure_priority': False, # Force constant FPS

            # Color Stream QoS
            'rgb_camera.color_qos': 'SENSOR_DATA',

            # IMU QoS (Must set both to ensure the combined 'imu' topic is stable)
            'gyro_qos': 'SENSOR_DATA',
            'accel_qos': 'SENSOR_DATA',

            # Stereo QoS - SENSOR_DATA (BEST_EFFORT) to match VSLAM subscriber
            'infra1_qos': 'SENSOR_DATA',
            'infra2_qos': 'SENSOR_DATA',
            'infra1_info_qos': 'SENSOR_DATA',
            'infra2_info_qos': 'SENSOR_DATA',

            'depth_module.depth_frame_id': 'camera_infra1_optical_frame',
            # these don't do anything - the second is the target
            # we have a script node - frame_rename.py that takes care of renaming frames
            'infra1_frame_id': 'camera_infra1_optical_frame',
            'infra2_frame_id': 'camera_infra2_optical_frame',
        }],
        remappings=[
            # ('imu', '/camera/imu'), 
        ],
    )

    # 5. The Integrated Single Container
    # This combines key nodes into one process for zero-copy memory sharing.
    # Use multi-threaded container to allow parallel processing of stereo images.
    # Single-threaded container processes callbacks sequentially which may break
    # cuVSLAM's internal stereo sync.
    isaac_ros_container = ComposableNodeContainer(
        name='isaac_ros_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',     # Multi-threaded executor
        composable_node_descriptions=[
            realsense_node, 
            # vslam_node, 
            # nvblox_node
            ],
        output='screen',
    )

    # 6. Foxglove
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'address': '0.0.0.0',
            'port': 8765,
            'use_compression': True,
            'topic_whitelist': [
                            '^/tf', 
                            '/tf_static',
                            '/diagnostics.*',
                            '/color/image_raw/compressed',
                        ],
            'send_buffer_limit': 10000000,
            'min_qos_depth': 1,
            'max_qos_depth': 10
        }],
    )

    return LaunchDescription([
        foxglove_bridge_node,
        isaac_ros_container,
    ])