#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Arguments
    run_foxglove_arg = DeclareLaunchArgument(
        'run_foxglove', default_value='false',
        description='Run Foxglove bridge'
    )
    
    enable_visual_slam_arg = DeclareLaunchArgument(
        'enable_visual_slam', default_value='true',
        description='Enable Isaac ROS Visual SLAM'
    )

    # 2. Robot State Publisher
    urdf_path = PathJoinSubstitution([
        FindPackageShare('rc_hardware_control'),
        'description', 'description.urdf.xacro'
    ])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(launch.substitutions.Command(['xacro ', urdf_path]), value_type=str),
            'use_sim_time': False,
            'publish_frequency': 30.0, # Force regular updates
            'ignore_timestamp': False,  # Keep this false to ensure VSLAM gets timed data
        }]
    )

    # 3. RealSense Camera
    # this used to be in it's own file, but here now so can be combined
    # with vslam and nvblox to share memory
    default_profile = '640x480x30'

    realsense_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='camera',
        namespace='',
        parameters=[{
            'camera_name': 'camera',
            'host_performance_step': True,

            # FORCE SYSTEM TIME (Disable hardware timestamps)
            'global_time_enabled': False,   # force to use ros::Time::now()
            'depth_module.global_time_enabled': False,
            'stereo_module.global_time_enabled': False, # Explicit module override
            'motion_module.global_time_enabled': False, # Explicit module override
            'rgb_camera.global_time_enabled': False,    # Explicit module override
        
            # Ensure frame IDs match your URDF exactly
            'base_frame_id': 'camera_link',
            'depth_module.depth_frame_id': 'camera_infra1_optical_frame',
            'infra1_frame_id': 'camera_infra1_optical_frame',
            'infra2_frame_id': 'camera_infra2_optical_frame',

            # 1. TF Management - Set to False because we use our own URDF
            'publish_tf': False, 
            'tf_publish_rate': 30.0, # Force dynamic TFs for the IMU/Optical frames
            'camera_base_frame': 'camera_link',

            # 2. Profiles (Modern D400 Parameter Naming)
            # Must set BOTH depth_module.profile AND depth_module.infra_profile
            # librealsense2 does not set default profiles for infra streams!
            'depth_module.depth_profile': '640x480x15',     # 15 hz plenty for mapping
            'depth_module.infra_profile': '640x480x30',
            'depth_module.color_profile': '640x480x30',

            # 3. Enable Streams for VSLAM & NVBLOX
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_depth': True,   # Required for nvblox
            'enable_color': False,  # Save USB bandwidth (VSLAM/Nvblox don't use it in this config)
            'enable_sync': True,    # Disable strict sync to prevent frame drops causing "sleep"

            # 4. IMU Configuration (Crucial for cuVSLAM)
            'gyro_fps': 200,
            'accel_fps': 250,               # D435i supports 63 or 250Hz - use 250 for VIO
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2,          # 2 = Linear Interpolation (Standard for VIO)

            # 5. Performance & Stability
            'initial_reset': False,        # Enable reset to clear IMU calibration errors
            'reconnect_timeout': 6.0,      # Wait seconds before trying to reconnect
            'wait_for_device_timeout': 30.0, # Wait for device to become available
            # 'depth_module.emitter_enabled': 0, # Set to 0 if outdoors
            'depth_module.emitter_enabled': 0, # 0 = Disabled (Fixes "Zero Poses" by removing static projection)
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
            # Ensure these match what your VSLAM and nvblox are expecting
        ]
    )

    # realsense_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('isaac_ros_realsense_control'),
    #             # 'launch', 'realsense_basic.launch.py'
    #             'launch', 'realsense_d435i.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         # Use 640x480x30 - a supported D435i resolution
    #         'infra_profile': '640x480x30',
    #         'enable_depth': 'false',      # Disabled to reduce USB load - VSLAM only needs stereo IR
    #         'enable_sync': 'true',        # CRITICAL: Must sync stereo frames for VSLAM
    #         'emitter_enabled': '0',       # Disable emitter to prevent VSLAM confusion
    #         'enable_accel': 'true',      
    #         'enable_gyro': 'true',       
    #         'unite_imu_method': '2',
    #         'initial_reset': 'false',   # Disabled - causes USB disconnect on Jetson
    #     }.items()   
    # )

    # 4. Define Composable Nodes (VSLAM + NVBLOX)
    vslam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'tf_buffer_duration_sec': 30.0,
            
            'publish_odom_to_base_tf': True,
            'publish_map_to_odom_tf': True,
            'base_frame': 'base_link',      # not camera_link ???

            # Frame IDs for RealSense D435i (new API style)
            'imu_frame': 'camera_gyro_optical_frame',
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],

            'enable_image_denoising': True,    # from default isaac ros launch
            'enable_imu_fusion': True,   # Disabled - IMU has "Motion Module force pause" errors
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,

            'image_jitter_threshold_ms': 40.0,  
            'sync_matching_threshold_ms': 5.0,   
            'num_cameras': 2,                     # Stereo mode
            'multicam_mode': 1,                   # 1 = stereo mode
            'min_num_images': 2,                  # Require both stereo images for proper matching

            'enable_observations_view': True,
            'enable_landmarks_view': True,
            'enable_slam_visualization': True,

            'rectified_images': True,  # D435i publishes rectified images
            'map_frame': 'map',
            'odom_frame': 'odom',

            # Ensure this matches your D435i's IR output
            # was camera_infra...
            'input_left_camera_frame': 'camera_infra1_optical_frame',
            'input_right_camera_frame': 'camera_infra2_optical_frame',

        }],
        remappings=[
            # Use synced topics - our republisher fixes the frame_id bug
            ('visual_slam/image_0', '/infra1/image_rect_raw'),
            ('visual_slam/image_1', '/infra2/image_rect_raw'),
            
            # Camera Info (Metadata)
            ('visual_slam/camera_info_0', '/infra1/camera_info'),
            ('visual_slam/camera_info_1', '/infra2/camera_info_fixed'),  # renamed frame to camera_infra2_optical_frame
            
            # IMU
            ('visual_slam/imu', '/imu')
        ],
    )

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        parameters=[{
            'global_frame': 'odom',
            'pose_frame': 'base_link',
            'use_tf_transforms': True,     
            'use_topic_transforms': False, 

            # ESDF Slicing (Crucial for an RC Truck)
            'voxel_size': 0.10,
            'static_mapper.esdf_slice_height': 0.10,    # The height Nav2 "looks" at
            'static_mapper.esdf_slice_min_height': 0.0, # Objects from ground up...
            'static_mapper.esdf_slice_max_height': 1.0, # ...to 0.5m high are obstacles
            'max_mapping_distance_m': 5.0,
            'map_clearing_radius_m': 5.0,  # Ensure this is a positive, non-zero number
            'map_clearing_frame_id': 'base_link',

            'use_depth': True,
            'use_color': False,
            'use_lidar': False,
            
            # QoS - Crucial for RealSense stability over USB
            'base_config_type': 'realsense',
            'input_qos': 'SENSOR_DATA',
            
            # Mapping Rates
            'integrate_depth_rate_hz': 10.0, # Slow down integration to save GPU
            'update_mesh_rate_hz': 2.0,
            'update_esdf_rate_hz': 5.0,      # Rate for the Nav2 costmap

            
            # Max range for depth integration (Prevents far-off noise from ruining the map)
            'static_mapper.projective_integrator_max_integration_distance_m': 5.0,
            
            'static_occupancy_2d': True,

            'transform_lookup_buffer_duration_sec': 0.5,

        }],
        remappings=[
            ('camera_0/depth/image', '/camera/depth/image_rect_raw'),
            ('camera_0/depth/camera_info', '/camera/depth/camera_info'),
            # Pose remappings - try both the namespaced and direct version
            # ('camera_0/pose', '/visual_slam/tracking/vo_pose'), 
            ('pose', '/visual_slam/tracking/vo_pose'), 
        ],
    )       

    # 6. Foxglove
    foxglove_bridge_node = Node(
        condition=IfCondition(LaunchConfiguration('run_foxglove')),
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
                            '^/visual_slam/.*',
                            '^/nvblox_node/.*',
                            '/nvblox_node/mesh',
                            '.*/compressed$'
                        ],
            'send_buffer_limit': 10000000,
            'min_qos_depth': 1,
            'max_qos_depth': 10
        }],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        # env={"RCUTILS_LOGGING_BUFFERED_STREAM": "1", "RCUTILS_LOGGING_SEVERITY": "DEBUG"}
    )

    # Jetson Stats Node - publishes on top of /diagnostics
    jetson_stats_node = Node(
        package='isaac_ros_jetson_stats',
        executable='jtop',
        name='jetson_stats_node',
        output='screen',
        parameters=[{'interval': 0.5}],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        # env={"RCUTILS_LOGGING_BUFFERED_STREAM": "1", "RCUTILS_LOGGING_SEVERITY": "DEBUG"}
    )


    # Add this alongside your other nodes
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['/joint_states']}],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        # env={"RCUTILS_LOGGING_BUFFERED_STREAM": "1", "RCUTILS_LOGGING_SEVERITY": "DEBUG"}
    )

    # Stereo Sync Republisher - Ensures perfect stereo synchronization for VSLAM
    # stereo_sync_node = Node(
    #     package='isaac_ros_realsense_control',
    #     executable='stereo_sync_republisher.py',
    #     name='stereo_sync_republisher',
    #     output='screen',
    # )

    # The realsense frame_id fixer node
    frame_rename_node = Node(
        package='isaac_ros_realsense_control',
        executable='frame_rename.py',
        name='frame_rename',
        output='screen',
    )

    # 5. The Single Container (The Fix)
    # This combines both nodes into one process for zero-copy memory sharing.
    # Use multi-threaded container to allow parallel processing of stereo images.
    # Single-threaded container processes callbacks sequentially which may break
    # cuVSLAM's internal stereo sync.
    isaac_ros_container = ComposableNodeContainer(
        name='isaac_ros_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',     # Multi-threaded executor
        composable_node_descriptions=[realsense_node, vslam_node, nvblox_node],
        output='screen',
    )


    return LaunchDescription([
        enable_visual_slam_arg,
        run_foxglove_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        jetson_stats_node,
        # republish_node,   # was just 'node', not composable
        foxglove_bridge_node,

        # Start frame rename after camera is up (fixes frame_id bug)
        TimerAction(
            period=2.0,
            actions=[frame_rename_node]
        ),

        # Start the VSLAM container after frame rename is running
        TimerAction(
            period=5.0,
            actions=[isaac_ros_container]
        ),

    ])