#!/usr/bin/env python3
"""Perception bring-up: RealSense D435i, visual SLAM and nvblox in one container.

Interface (launch arguments):
  camera_profile            stereo stream WxHxFPS, e.g. 848x480x30
  obstacle_band_lower_edge  metres above the robot frame where nvblox's 2D
                            slice starts; anything above it is an obstacle
  robot_frame               the frame visual SLAM tracks and nvblox clears around

Included by rccarauto.launch.py (full stack) and perception_only.launch.py
(camera bench). IMU fusion is pinned off: the D435i IMU is not streamed and
cuVSLAM runs stereo-only, which is what produced the last working map.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

# 848x480x30 is the stereo profile that produced the last working map. Other
# D435i options: 640x480x30, 424x240x30, 320x240x30.
DEFAULT_CAMERA_PROFILE = '848x480x30'
# Colour is disabled below; this profile only matters if it is re-enabled.
COLOR_PROFILE = '640x480x15'
# nvblox slices its 2D map from the voxel rows between the band's lower and
# upper edges (edges floored to a row, rows inclusive) and marks a voxel as an
# obstacle when its TSDF distance is within one voxel of a surface. The row
# touching the floor therefore always registers the floor, so the lower edge
# must fall in row one or higher. With 5 cm rows, 0.06 selects the row from
# 0.05 to 0.10 m: its centre is 7.5 cm up, so the floor is 2.5 cm outside the
# obstacle test and anything with a top above about 2.5 cm registers, which
# covers everything the 3.8 cm ground clearance cannot pass over. The upper
# edge is chassis height. test_vehicle_geometry.py asserts the row arithmetic.
NVBLOX_VOXEL_SIZE = 0.05
DEFAULT_OBSTACLE_BAND_LOWER_EDGE = 0.06
OBSTACLE_BAND_UPPER_EDGE = 0.50
DEFAULT_ROBOT_FRAME = 'base_footprint'

# The camera and its info republisher start first; the container follows once
# the fixed camera_info topic exists, so visual SLAM never sees the wrong frame.
CONTAINER_START_DELAY_S = 4.0


def perception_nodes(camera_profile, obstacle_band_lower_edge, robot_frame):
    """The three composable nodes. Arguments may be plain values or substitutions."""
    camera = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        namespace='',
        name='camera',
        parameters=[{
            'camera_name': 'camera',
            # Force ROS time instead of hardware timestamps, on every module.
            'global_time_enabled': False,
            'depth_module.global_time_enabled': False,
            'stereo_module.global_time_enabled': False,
            'motion_module.global_time_enabled': False,
            'rgb_camera.global_time_enabled': False,
            'host_performance_step': 'true',

            # TF comes from the URDF, not the driver.
            'publish_tf': False,
            'tf_publish_rate': 30.0,
            'camera_base_frame': 'camera_link',

            # librealsense sets no default for the infra streams, so both the
            # depth profile and the infra profile must be given.
            'depth_module.profile': camera_profile,
            'depth_module.infra_profile': camera_profile,
            'rgb_camera.profile': COLOR_PROFILE,

            # Streams: stereo infra for visual SLAM, depth for nvblox. Colour is
            # off to save USB bandwidth; it caused a hardware crash when on.
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_depth': True,
            'enable_color': False,
            'enable_sync': True,   # stereo frames must be matched for cuVSLAM

            # IMU off: fusion is pinned off in visual SLAM below.
            'gyro_fps': 200,
            'accel_fps': 250,
            'enable_gyro': False,
            'enable_accel': False,
            'unite_imu_method': 1,

            # A hardware reset re-enumerates the camera. The container could not
            # follow that until /dev was bind-mounted live (issue #9). Kept off;
            # re-enabling it is a separate decision.
            'initial_reset': False,
            'reconnect_timeout': 6.0,
            'wait_for_device_timeout': 30.0,
            'depth_module.emitter_enabled': 0,   # 0 outdoors; the IR projector fights sunlight
            'depth_module.depth_qos': 'SENSOR_DATA',
            'depth_module.exposure_priority': False,   # constant FPS over exposure

            'rgb_camera.color_qos': 'SENSOR_DATA',
            'gyro_qos': 'SENSOR_DATA',
            'accel_qos': 'SENSOR_DATA',

            # BEST_EFFORT to match the visual SLAM subscriber.
            'infra1_qos': 'SENSOR_DATA',
            'infra2_qos': 'SENSOR_DATA',
            'infra1_info_qos': 'SENSOR_DATA',
            'infra2_info_qos': 'SENSOR_DATA',

            'depth_module.depth_frame_id': 'camera_infra1_optical_frame',
            # The driver ignores the infra frame_id parameters; frame_rename.py
            # republishes infra2's camera_info with the frame visual SLAM expects.
        }],
        remappings=[
            ('imu', '/imu'),
            ('infra1/image_rect_raw', '/infra1/image_rect_raw'),
            ('infra2/image_rect_raw', '/infra2/image_rect_raw'),
            ('infra1/camera_info', '/infra1/camera_info'),
            ('infra2/camera_info', '/infra2/camera_info'),
            ('depth/image_rect_raw', '/depth/image_rect_raw'),
            ('depth/camera_info', '/depth/camera_info'),
            ('color/image_raw', '/color/image_raw'),
            ('color/image_raw/compressed', '/color/image_raw/compressed'),
            ('color/camera_info', '/color/camera_info'),
        ],
    )

    vslam = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'tf_buffer_duration_sec': 30.0,

            'publish_odom_to_base_tf': True,
            'publish_map_to_odom_tf': True,
            'base_frame': robot_frame,

            'imu_frame': 'camera_gyro_optical_frame',
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],

            'enable_image_denoising': True,
            'enable_imu_fusion': False,   # pinned off; stereo-only tracking
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,

            'image_jitter_threshold_ms': 60.0,
            'sync_matching_threshold_ms': 5.0,
            'num_cameras': 2,
            'multicam_mode': 1,       # stereo
            'min_num_images': 2,      # both images or no update

            'enable_observations_view': True,
            'enable_landmarks_view': True,
            'enable_slam_visualization': True,

            'rectified_images': True,   # the D435i publishes rectified infra
            'map_frame': 'map',
            'odom_frame': 'odom',

            'input_left_camera_frame': 'camera_infra1_optical_frame',
            'input_right_camera_frame': 'camera_infra2_optical_frame',
        }],
        remappings=[
            ('visual_slam/image_0', '/infra1/image_rect_raw'),
            ('visual_slam/image_1', '/infra2/image_rect_raw'),
            ('visual_slam/camera_info_0', '/infra1/camera_info'),
            # frame_rename.py's copy with frame_id camera_infra2_optical_frame
            ('visual_slam/camera_info_1', '/infra2/camera_info_fixed'),
            ('visual_slam/imu', '/imu'),
        ],
    )

    nvblox = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        parameters=[{
            'global_frame': 'odom',   # ADR-0001: map in odom, never in map
            'pose_frame': robot_frame,
            'use_tf_transforms': True,
            'use_topic_transforms': False,

            # ESDF slice: the 2D band Nav2's costmaps read (see the note at the top).
            'voxel_size': NVBLOX_VOXEL_SIZE,
            'static_mapper.esdf_slice_height': 0.20,   # z of the published slice, not a band edge
            'static_mapper.esdf_slice_min_height': ParameterValue(obstacle_band_lower_edge, value_type=float),
            'static_mapper.esdf_slice_max_height': OBSTACLE_BAND_UPPER_EDGE,
            'max_mapping_distance_m': 5.0,
            'map_clearing_radius_m': 5.0,
            'map_clearing_frame_id': robot_frame,

            'use_depth': True,
            'use_color': False,
            'use_lidar': False,

            'base_config_type': 'realsense',
            'input_qos': 'SENSOR_DATA',

            'integrate_depth_rate_hz': 10.0,   # eases the GPU
            'update_mesh_rate_hz': 2.0,
            'update_esdf_rate_hz': 5.0,        # feeds the Nav2 costmaps

            'static_mapper.projective_integrator_max_integration_distance_m': 5.0,

            'static_occupancy_2d': True,

            'transform_lookup_buffer_duration_sec': 0.5,
        }],
        remappings=[
            ('camera_0/depth/image', '/depth/image_rect_raw'),
            ('camera_0/depth/camera_info', '/depth/camera_info'),
            ('pose', '/visual_slam/tracking/vo_pose'),
        ],
    )

    return camera, vslam, nvblox


def generate_launch_description():
    camera_profile = LaunchConfiguration('camera_profile')
    obstacle_band_lower_edge = LaunchConfiguration('obstacle_band_lower_edge')
    robot_frame = LaunchConfiguration('robot_frame')

    camera, vslam, nvblox = perception_nodes(camera_profile, obstacle_band_lower_edge, robot_frame)

    # Republishes infra2's camera_info with the frame id visual SLAM expects.
    frame_rename_node = Node(
        package='rc_hardware_control',
        executable='frame_rename.py',
        name='frame_rename',
        output='screen',
    )

    # One multi-threaded container so the three nodes share images zero-copy
    # and cuVSLAM's stereo sync is not serialised behind other callbacks.
    container = ComposableNodeContainer(
        name='isaac_ros_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[camera, vslam, nvblox],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_profile', default_value=DEFAULT_CAMERA_PROFILE,
                              description='Stereo infra and depth profile, WxHxFPS'),
        DeclareLaunchArgument('obstacle_band_lower_edge', default_value=str(DEFAULT_OBSTACLE_BAND_LOWER_EDGE),
                              description='Metres above robot_frame where the nvblox 2D slice starts'),
        DeclareLaunchArgument('robot_frame', default_value=DEFAULT_ROBOT_FRAME,
                              description='Frame visual SLAM tracks and nvblox clears around'),
        frame_rename_node,
        TimerAction(period=CONTAINER_START_DELAY_S, actions=[container]),
    ])
