#!/usr/bin/env python3
"""Perception bring-up: RealSense D435i, emitter splitter, visual SLAM and nvblox in one container.

Interface (launch arguments):
  camera_profile            stereo stream WxHxFPS, e.g. 848x480x30
  obstacle_band_lower_edge  metres above the odom origin where nvblox's 2D
                            slice starts. odom's z is zero where visual SLAM
                            started, so on level ground this is height above
                            the floor; on a slope the band tilts with odom,
                            not with the car. See vehicle_geometry.py for the
                            voxel-row arithmetic behind the default.
  robot_frame               the frame visual SLAM tracks and nvblox clears around
  camera_reset              true: hardware-reset the camera before streaming.
                            The perception watchdog sets it on a start that
                            follows a start which never came up.

Run by the perception watchdog in rccarauto.launch.py (full stack), which
restarts it when it stalls or exits, and included by perception_only.launch.py
(camera bench). IMU fusion is pinned off: the D435i IMU is not streamed and
cuVSLAM runs stereo-only, which is what produced the last working map.
"""
from ament_index_python import get_resource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

from rc_hardware_control.vehicle_geometry import (
    NVBLOX_VOXEL_SIZE, OBSTACLE_BAND_LOWER_EDGE, OBSTACLE_BAND_UPPER_EDGE)

# 848x480x30 is the stereo profile that produced the last working map. Other
# D435i options: 640x480x30, 424x240x30, 320x240x30.
DEFAULT_CAMERA_PROFILE = '848x480x30'
# Colour is disabled below; this profile only matters if it is re-enabled.
COLOR_PROFILE = '640x480x15'
# Obstacle band defaults come from vehicle_geometry.py, where the voxel-row
# arithmetic is explained and test_vehicle_geometry.py asserts it: with 5 cm
# rows, 0.06 selects the row from 0.05 to 0.10 m, the floor stays out, and
# anything with a top above about 2.5 cm registers.
DEFAULT_OBSTACLE_BAND_LOWER_EDGE = OBSTACLE_BAND_LOWER_EDGE
DEFAULT_ROBOT_FRAME = 'base_footprint'

# The camera and its info republisher start first; the container follows once
# the fixed camera_info topic exists, so visual SLAM never sees the wrong frame.
CONTAINER_START_DELAY_S = 4.0

# The splitter's outputs (issue #15): what visual SLAM and nvblox read.
EMITTER_OFF_INFRA1 = '/emitter_off/infra1/image_rect_raw'
EMITTER_OFF_INFRA2 = '/emitter_off/infra2/image_rect_raw'
EMITTER_ON_DEPTH = '/emitter_on/depth/image_rect_raw'


def perception_nodes(camera_profile, obstacle_band_lower_edge, robot_frame, camera_reset=False):
    """The four composable nodes: camera, splitter, visual SLAM, nvblox. Arguments may be plain values or substitutions."""
    camera = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        namespace='',
        name='camera',
        # Names are those of the apt realsense2_camera 4.56.4, which matches the
        # image's librealsense 2.56.4 (issue #14). The driver ignores a name it
        # does not declare, without a warning: check `ros2 param dump /camera`.
        parameters=[{
            'camera_name': 'camera',
            # Force ROS time instead of hardware timestamps, on every module.
            # Stamps stay in ROS time even though the driver warns that the
            # frames' time domain is HARDWARE_CLOCK.
            'depth_module.global_time_enabled': False,
            'motion_module.global_time_enabled': False,
            'rgb_camera.global_time_enabled': False,

            # TF comes from the URDF, not the driver.
            'publish_tf': False,

            # librealsense sets no default for the infra streams, so both the
            # depth profile and the infra profile must be given.
            'depth_module.depth_profile': camera_profile,
            'depth_module.infra_profile': camera_profile,
            'rgb_camera.color_profile': COLOR_PROFILE,

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

            # A hardware reset re-enumerates the camera, which the container
            # follows through its live /dev mount (issue #9). Off by default:
            # it costs a few seconds. The perception watchdog turns it on for a
            # start after a failed start (issue #14): a camera wedged by a USB
            # reset failed every plain restart on the bench.
            'initial_reset': ParameterValue(camera_reset, value_type=bool),
            'reconnect_timeout': 6.0,
            'wait_for_device_timeout': 30.0,
            # Depth quality, settled on the bench 2026-09-14 (issue #5 comments):
            # the ceiling light and window saturate the top rows of the infra
            # images and stereo matching then invents returns at 0.2 to 0.3 m
            # there in about 2% of frames, enough to hold a phantom obstacle in
            # front of the car against nvblox's decay. Outdoors the sun or sky
            # does the same. The High Accuracy preset (3) raises the stereo
            # confidence threshold and removed every such return in 455 frames.
            # The temporal filter takes out single-frame outliers cheaply. The
            # spatial filter stays off: too costly on the CPU.
            #
            # The IR emitter alternates frame by frame (issue #15). Passive
            # stereo cannot range a plain wall, which stays a depth hole and so
            # unknown in the costmap; the emitter's dot pattern gives it texture
            # (valid depth 55% to 79% on the bench). The same dots in the infra
            # images disturb visual SLAM's tracking, so the splitter below sends
            # emitter-off infra frames to visual SLAM and emitter-on depth
            # frames to nvblox, each at half the camera rate. On the bench the
            # emitter also painted near returns on things close to the lens;
            # recheck that with the glare test from #5.
            'depth_module.visual_preset': 3,
            'depth_module.emitter_enabled': 1,
            'depth_module.emitter_on_off': True,
            'temporal_filter.enable': True,
            'spatial_filter.enable': False,
            'depth_qos': 'SENSOR_DATA',
            'color_qos': 'SENSOR_DATA',
            'gyro_qos': 'SENSOR_DATA',
            'accel_qos': 'SENSOR_DATA',

            # BEST_EFFORT to match the visual SLAM subscriber.
            'infra1_qos': 'SENSOR_DATA',
            'infra2_qos': 'SENSOR_DATA',
            'infra1_info_qos': 'SENSOR_DATA',
            'infra2_info_qos': 'SENSOR_DATA',

            # Frame ids are fixed by the driver as camera_<stream>_optical_frame,
            # as in the URDF, except that infra2's camera_info carries infra1's
            # frame; frame_rename.py republishes it with the frame visual SLAM
            # expects.
        }],
        # The driver publishes under its node name (/camera/depth/image_rect_raw),
        # so each rule names that full topic; a relative name matches nothing.
        # The compressed variants image_transport adds are not remapped and stay
        # under /camera (e.g. /camera/infra1/image_rect_raw/compressed).
        remappings=[
            ('/camera/imu', '/imu'),
            ('/camera/infra1/image_rect_raw', '/infra1/image_rect_raw'),
            ('/camera/infra2/image_rect_raw', '/infra2/image_rect_raw'),
            ('/camera/infra1/camera_info', '/infra1/camera_info'),
            ('/camera/infra2/camera_info', '/infra2/camera_info'),
            ('/camera/depth/image_rect_raw', '/depth/image_rect_raw'),
            ('/camera/depth/camera_info', '/depth/camera_info'),
            ('/camera/color/image_raw', '/color/image_raw'),
            ('/camera/color/camera_info', '/color/camera_info'),
        ],
    )

    # Republishes infra images from emitter-off frames and depth from emitter-on
    # frames, reading frame_emitter_mode from each frame's metadata. NVIDIA's
    # node, vendored in src/RCCar/realsense_splitter. camera_info is not split:
    # visual SLAM and nvblox take it from the camera, as in NVIDIA's examples.
    splitter = ComposableNode(
        name='realsense_splitter_node',
        namespace='',
        package='realsense_splitter',
        plugin='nvblox::RealsenseSplitterNode',
        parameters=[{
            'input_qos': 'SENSOR_DATA',
            'output_qos': 'SENSOR_DATA',
        }],
        remappings=[
            ('input/infra_1', '/infra1/image_rect_raw'),
            ('input/infra_1_metadata', '/camera/infra1/metadata'),
            ('input/infra_2', '/infra2/image_rect_raw'),
            ('input/infra_2_metadata', '/camera/infra2/metadata'),
            ('input/depth', '/depth/image_rect_raw'),
            ('input/depth_metadata', '/camera/depth/metadata'),
            ('/realsense_splitter_node/output/infra_1', EMITTER_OFF_INFRA1),
            ('/realsense_splitter_node/output/infra_2', EMITTER_OFF_INFRA2),
            ('/realsense_splitter_node/output/depth', EMITTER_ON_DEPTH),
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

            # Visual SLAM gets every other camera frame (emitter-off only), so
            # frames are 67 ms apart at 15 Hz; this allows one and a half periods.
            'image_jitter_threshold_ms': 100.0,
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
            ('visual_slam/image_0', EMITTER_OFF_INFRA1),
            ('visual_slam/image_1', EMITTER_OFF_INFRA2),
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
            'esdf_mode': '2d',   # nvblox's default; stated because the costmaps depend on it
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

            # Forgetting. nvblox decays unobserved voxels' weights but clamps
            # them at tsdf_decayed_weight_threshold (0.001), which is above
            # the ESDF's minimum weight for a surface (1e-4), so a removed
            # obstacle whose background is a depth hole stayed an obstacle for
            # ever (bench, 2026-09-14: a box held at 1 m never cleared in 90 s).
            # With this flag a fully decayed voxel's distance is set to free.
            # Decay 0.85 at 5 Hz clears a max-weight voxel in about 10 s; the
            # cost is that an obstacle out of view for that long is forgotten
            # as free, not unknown, until the camera sees it again. Faster
            # decay means quicker clearing of a vanished obstacle and quicker
            # forgetting of a real one the car has turned away from. Read at
            # start only; a param set at runtime has no effect.
            'static_mapper.tsdf_set_free_distance_on_decayed': True,
            'static_mapper.tsdf_decay_factor': 0.85,
            # Deallocating fully decayed blocks would make them unknown again
            # and nvblox warns every tick that the two flags conflict.
            'static_mapper.decay_integrator_deallocate_decayed_blocks': False,

            'transform_lookup_buffer_duration_sec': 0.5,
        }],
        remappings=[
            ('camera_0/depth/image', EMITTER_ON_DEPTH),
            ('camera_0/depth/camera_info', '/depth/camera_info'),
            ('pose', '/visual_slam/tracking/vo_pose'),
        ],
    )

    return camera, splitter, vslam, nvblox


def missing_components(nodes, context, lookup=get_resource):
    """Why the container could not load each node: package or plugin not in the ament index.

    The container resolves plugins through the ament index of the environment
    it inherits. A package built after that shell was sourced is missing, and
    the container only logs "Could not find requested resource in ament index"
    and runs on without the node.
    """
    problems = []
    for node in nodes:
        package = perform_substitutions(context, node.package)
        plugin = perform_substitutions(context, node.node_plugin)
        try:
            registered, _ = lookup('rclcpp_components', package)
        except LookupError:
            problems.append(f'{package} is not in the ament index')
            continue
        if plugin not in (line.split(';')[0] for line in registered.splitlines()):
            problems.append(f'{package} does not register {plugin}')
    return problems


def generate_launch_description():
    camera_profile = LaunchConfiguration('camera_profile')
    obstacle_band_lower_edge = LaunchConfiguration('obstacle_band_lower_edge')
    robot_frame = LaunchConfiguration('robot_frame')
    camera_reset = LaunchConfiguration('camera_reset')

    camera, splitter, vslam, nvblox = perception_nodes(
        camera_profile, obstacle_band_lower_edge, robot_frame, camera_reset)

    # Republishes infra2's camera_info with the frame id visual SLAM expects.
    frame_rename_node = Node(
        package='rc_hardware_control',
        executable='frame_rename.py',
        name='frame_rename',
        output='screen',
    )

    # One multi-threaded container so the four nodes share images zero-copy
    # and cuVSLAM's stereo sync is not serialised behind other callbacks.
    container = ComposableNodeContainer(
        name='isaac_ros_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[camera, splitter, vslam, nvblox],
        output='screen',
    )

    # Before anything starts: a node the container cannot load ends the launch
    # at once with the reason, instead of a container running without it while
    # the perception watchdog waits out its startup grace.
    def check_components(context):
        problems = missing_components([camera, splitter, vslam, nvblox], context)
        if problems:
            raise RuntimeError(
                'perception cannot start: ' + '; '.join(problems) + '. If a package was built '
                'after this shell was sourced, run `source install/setup.bash` and launch again.')
        return []

    # A container that dies ends the whole bring-up (a camera that never
    # appears ends in a segfault), so the perception watchdog sees an exit and
    # restarts at once instead of waiting for the streams to time out.
    end_with_container = RegisterEventHandler(OnProcessExit(
        target_action=container,
        on_exit=[EmitEvent(event=Shutdown(reason='perception container exited'))],
    ))

    return LaunchDescription([
        OpaqueFunction(function=check_components),
        DeclareLaunchArgument('camera_profile', default_value=DEFAULT_CAMERA_PROFILE,
                              description='Stereo infra and depth profile, WxHxFPS'),
        DeclareLaunchArgument('obstacle_band_lower_edge', default_value=str(DEFAULT_OBSTACLE_BAND_LOWER_EDGE),
                              description='Metres above robot_frame where the nvblox 2D slice starts'),
        DeclareLaunchArgument('robot_frame', default_value=DEFAULT_ROBOT_FRAME,
                              description='Frame visual SLAM tracks and nvblox clears around'),
        DeclareLaunchArgument('camera_reset', default_value='false',
                              description='Hardware-reset the camera before streaming'),
        frame_rename_node,
        end_with_container,
        TimerAction(period=CONTAINER_START_DELAY_S, actions=[container]),
    ])
