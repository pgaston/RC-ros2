#!/usr/bin/env python3
"""Camera bench: perception bring-up alone, with no ros2_control and no Nav2.

Use it to debug the RealSense, visual SLAM and nvblox without the car moving,
for example the camera re-enumeration in issue #9. The URDF is still published
so the transform from the robot frame to the camera optical frames exists.
perception.launch.py's arguments (camera_profile, obstacle_band_lower_edge,
robot_frame) can be given on the command line and reach the include unchanged.
For a viewer, run foxglove_bridge separately:
`ros2 launch foxglove_bridge foxglove_bridge_launch.xml`.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('rc_hardware_control')
    urdf_path = PathJoinSubstitution([share, 'description', 'description.urdf.xacro'])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str),
            'use_sim_time': False,
            'publish_frequency': 30.0,
        }],
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'perception.launch.py'])),
    )

    return LaunchDescription([
        robot_state_publisher_node,
        perception,
    ])
