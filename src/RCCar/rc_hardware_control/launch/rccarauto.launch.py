#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """Generate stabilized launch for D435i on Orin Nano."""

    # 1. Robot State Publisher
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

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("rc_hardware_control"),
        "config",
        "steer_bot_hardware.yaml",
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': ParameterValue(launch.substitutions.Command(['xacro ', urdf_path]), value_type=str)},
            {'use_sim_time': False},
            robot_controllers
        ],
        output="both",
        emulate_tty=True,
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/bicycle_steering_controller/reference", "/cmd_vel"),
            ("/bicycle_steering_controller/reference_unstamped", "/cmd_vel"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    bicycle_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bicycle_steering_controller"],
    )

    delayed_bicycle_steering_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[bicycle_steering_controller_spawner],
        )
    )



    # 2. Consolidated Realsense Profiles (640x480 @ 30fps for stability)
    # Using 640x480 reduces USB bandwidth and decompression CPU cycles.
    # various resolutions:   848x480x30, 640x480x30, 424x240x30, 320x240x30
    # depth not used
    color_profile = '640x480x15'    # slower
    default_profile = '848x480x30'

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
            'enable_depth': True, 
            'enable_color': True, # Enabled for Foxglove visualization
            'enable_sync': True,  # MUST be true for stereo VSLAM to match frames properly
            
            # 4. IMU Configuration (Crucial for cuVSLAM)
            'gyro_fps': 200,
            'accel_fps': 250,  # D435i supports 63 or 250Hz - use 250 for VIO
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 1, # 1 = Copy (Standard for VIO)
            
            # 5. Performance & Stability
            'initial_reset': True,        # Enable reset to clear IMU/MIPI calibration errors
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
            # 'infra1_frame_id': 'camera_infra1_optical_frame',
            # 'infra2_frame_id': 'camera_infra2_optical_frame',
            # 'stereo_module.infra1_frame_id': 'infra1_optical_frame',
            # 'stereo_module.infra2_frame_id': 'infra2_optical_frame',
        }],
        remappings=[
            ('imu', '/imu'), 
        ],
    )

    # The realsense frame_id fixer node
    frame_rename_node = Node(
        package='rc_hardware_control',
        executable='frame_rename.py',
        name='frame_rename',
        output='screen',
    )

    # 4. Define Composable Nodes (VSLAM + NVBLOX)
    vslam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'tf_buffer_duration_sec': 30.0,
            
            'publish_odom_to_base_tf': True,
            'publish_map_to_odom_tf': True,
            'base_frame': 'base_link',      # Use base_link as the reference frame

            # Frame IDs for RealSense D435i (new API style)
            'imu_frame': 'camera_gyro_optical_frame',
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],

            'enable_image_denoising': True,    
            'enable_imu_fusion': False,   # Disabled so VSLAM doesn't get stuck waiting for IMU
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,

            'image_jitter_threshold_ms': 60.0,  
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
            'static_mapper.esdf_slice_height': 0.20,    # The height Nav2 "looks" at
            'static_mapper.esdf_slice_min_height': 0.15, # Start slicing ABOVE the floor (15cm)
            'static_mapper.esdf_slice_max_height': 1.0, # ...to 1.0m high are obstacles
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
            ('camera_0/depth/image', '/depth/image_rect_raw'),
            ('camera_0/depth/camera_info', '/depth/camera_info'),
            # Pose remappings - try both the namespaced and direct version
            # ('camera_0/pose', '/visual_slam/tracking/vo_pose'), 
            ('pose', '/visual_slam/tracking/vo_pose'), 
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
            vslam_node, 
            nvblox_node
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
                            '/plan',
                            '/local_plan',
                            '/cmd_vel',
                            '/goal_pose',
                            '/clicked_point',
                            '^/local_costmap/.*',
                            '^/global_costmap/.*',
                            '^/visual_slam/.*',
                            '^/nvblox_node/.*',
                            '/nvblox_node/mesh',

                        ],
            'send_buffer_limit': 100000000,
            'min_qos_depth': 1,
            'max_qos_depth': 10
        }],
    )

    # 7. Navigation2 (Nav2)
    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('rc_hardware_control'),
        'config', 'my_custom_nav2_params.yaml'
    ])
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': nav2_params_file,
            'use_bond': 'False'
        }.items()
    )

    goal_pose_relay_node = Node(
        package='rc_hardware_control',
        executable='goal_pose_relay.py',
        name='goal_pose_relay',
        output='screen',
        parameters=[{
            'default_goal_frame': 'odom',
            'action_name': 'navigate_to_pose',
        }],
    )


    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        delayed_bicycle_steering_controller_spawner,
        jetson_stats_node,
        frame_rename_node,
        foxglove_bridge_node,
        
        # Start the VSLAM container after frame rename is running
        TimerAction(
            period=4.0,
            actions=[isaac_ros_container]
        ),
        
        # Start Nav2 slightly after VSLAM to ensure TFs are ready
        TimerAction(
            period=8.0,
            actions=[nav2_launch, goal_pose_relay_node]
        ),
    ])