#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    """The full stack: URDF, ros2_control, perception, Foxglove, Nav2, goal relay."""

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
            'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str),
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
            {'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str)},
            {'use_sim_time': False},
            robot_controllers
        ],
        output="both",
        emulate_tty=True,
        remappings=[
            ("~/robot_description", "/robot_description"),
            # The Steering controller listens only to the velocity mux, never
            # to Nav2 or teleop directly (config/velocity_mux.yaml). Stamped
            # input; the mux publishes TwistStamped.
            ("/bicycle_steering_controller/reference", "/cmd_vel_mux"),
        ],
    )

    # Velocity mux: teleop over the perception hold over Nav2, zero when none is fresh.
    velocity_mux_node = Node(
        package='rc_hardware_control',
        executable='velocity_mux.py',
        name='velocity_mux',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('rc_hardware_control'), 'config', 'velocity_mux.yaml'
        ])],
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



    # 2. Perception bring-up (perception.launch.py: camera, visual SLAM, nvblox),
    # run by the perception watchdog as its child. The watchdog holds the car
    # through the velocity mux while depth, visual SLAM odometry or the
    # occupancy grid is stale, and restarts the bring-up when it stalls or
    # exits (config/perception_watchdog.yaml, issue #14). Perception arguments
    # (camera_profile, obstacle_band_lower_edge, robot_frame) keep their
    # defaults; to override one, append name:=value to the watchdog's command.
    # camera_reset is the watchdog's own: it adds it after a start that never
    # came up.
    # Respawned if it dies; the bring-up it ran is stopped with it.
    perception_watchdog_node = Node(
        package='rc_hardware_control',
        executable='perception_watchdog.py',
        name='perception_watchdog',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[PathJoinSubstitution([
            FindPackageShare('rc_hardware_control'), 'config', 'perception_watchdog.yaml'
        ])],
    )

    # 6. Foxglove
    # Over the car's wifi link (~7 Mbit/s, issue #10) Foxglove gets only what
    # fits: no full-rate image, and a 5 Hz half-width JPEG from the camera
    # preview instead (issue #16).
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'address': '0.0.0.0',
            'port': 8765,
            # Websocket compression costs a third of a core on the Orin Nano
            # (bench, 2026-09-14) and starves the perception container.
            'use_compression': False,
            # Per-client queue before the bridge drops messages. At 100 MB a
            # bridge or link that fell behind queued seconds of data and the
            # camera view ran 4 s late (2026-09-14); the 10 MB default is still
            # over 10 s of backlog on the wifi link, so 1 MB.
            'send_buffer_limit': 1000000,
            # Each pattern must match the whole topic name (std::regex_match).
            # The full-rate images are left out: the infra stream alone is about
            # 20 Mbit/s and raw depth 24 MB/s (2026-09-16). For those on the
            # bench, run a second bridge by hand (README).
            'topic_whitelist': [
                            '^/tf', 
                            '/tf_static',
                            '/diagnostics.*',
                            '/camera_preview/image/compressed',   # 5 Hz, 424 px wide
                            '/plan',
                            '/local_plan',
                            '/cmd_vel',          # Nav2's output
                            '/cmd_vel_teleop',   # the Foxglove Teleop panel publishes here
                            '/cmd_vel_mux',      # what the Steering controller receives
                            '/cmd_vel_hold',     # the perception watchdog's hold
                            '/perception/status',
                            '/goal_relay/status',
                            '/goal_relay/cancel',   # the stop button (Publish panel, std_msgs/String)
                            '/goal_pose',
                            '/clicked_point',
                            '^/local_costmap/.*',
                            '^/global_costmap/.*',
                            '^/visual_slam/.*',
                            '^/nvblox_node/.*',   # includes the mesh, ~30 Mbit/s: leave it out of remote layouts
                        ],
            # /tf best effort. With a reliable /tf reader here, both /tf writers
            # (robot_state_publisher, visual SLAM) at times kept resending to
            # the bridge, ~5000 samples a second each, and heartbeating every
            # /tf reader: ~85k packets a second, 60% of a core here and 45%
            # more in perception (issue #17, captured 2026-09-16). A best-effort
            # reader gets no resends. A lost /tf sample is replaced ~20 ms
            # later. /tf_static is latched and rare, so it stays reliable.
            'best_effort_qos_topic_whitelist': ['/tf'],
            'min_qos_depth': 1,
            'max_qos_depth': 10
        }],
    )

    # The camera view for Foxglove: visual SLAM's emitter-off infra1 stream,
    # which has no IR dots, cut to 5 Hz and 424 px wide (estimated under
    # 1 Mbit/s; measure on the car).
    # Outside the perception bring-up, so the watchdog's restarts leave it be;
    # it picks the stream up again when the camera returns.
    camera_preview_node = Node(
        package='rc_hardware_control',
        executable='camera_preview.py',
        name='camera_preview',
        output='screen',
        parameters=[{
            'input_topic': '/emitter_off/infra1/image_rect_raw',
            'output_topic': '/camera_preview/image/compressed',
            'rate_hz': 5.0,
            'max_width': 424,
            'jpeg_quality': 60,
        }],
    )

    # 7. Navigation2 (Nav2)
    # The behaviour tree parameters need absolute paths, so the params file is
    # rewritten with the installed location of the car-like tree. Both
    # navigators get it; see the note in the params file.
    car_like_tree = PathJoinSubstitution([
        FindPackageShare('rc_hardware_control'),
        'behavior_trees', 'navigate_to_pose_car_like.xml'
    ])
    nav2_params_file = RewrittenYaml(
        source_file=PathJoinSubstitution([
            FindPackageShare('rc_hardware_control'),
            'config', 'my_custom_nav2_params.yaml'
        ]),
        param_rewrites={
            'default_nav_to_pose_bt_xml': car_like_tree,
            'default_nav_through_poses_bt_xml': car_like_tree,
        },
        convert_types=True,
    )

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
            'use_bond': 'False',
            'log_level': 'WARN'
        }.items()
    )

    goal_pose_relay_node = Node(
        package='rc_hardware_control',
        executable='goal_pose_relay.py',
        name='goal_pose_relay',
        output='screen',
        # Admission, preemption and the status topic's vocabulary are in the
        # script's docstring.
        parameters=[{
            'global_frame': 'odom',          # ADR-0001
            'robot_frame': 'base_footprint',
            'action_name': 'navigate_to_pose',
            'status_topic': '/goal_relay/status',
            'occupancy_grid_topic': '/nvblox_node/static_occupancy_grid',   # what both costmaps read
            'perception_status_topic': '/perception/status',
        }],
    )


    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        delayed_bicycle_steering_controller_spawner,
        velocity_mux_node,
        jetson_stats_node,
        foxglove_bridge_node,
        camera_preview_node,
        perception_watchdog_node,

        # Start Nav2 after the perception container (up at 4 s) so TFs are ready
        TimerAction(
            period=8.0,
            actions=[nav2_launch, goal_pose_relay_node]
        ),
    ])