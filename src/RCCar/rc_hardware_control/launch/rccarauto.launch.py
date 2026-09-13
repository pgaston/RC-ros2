#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

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



    # 2. Perception bring-up: camera, visual SLAM and nvblox (perception.launch.py).
    # Its arguments (camera_profile, obstacle_band_lower_edge, robot_frame) keep
    # their defaults here; pass them on the command line to override.
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rc_hardware_control'),
                'launch', 'perception.launch.py'
            ])
        ),
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
                            '/infra1/image_rect_raw/compressed',
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
            'use_bond': 'False',
            'log_level': 'WARN'
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
        foxglove_bridge_node,
        perception,

        # Start Nav2 after the perception container (up at 4 s) so TFs are ready
        TimerAction(
            period=8.0,
            actions=[nav2_launch, goal_pose_relay_node]
        ),
    ])