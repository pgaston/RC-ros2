import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('pca9685_ros2_control_example')
    
    # 1. Path to your URDF and Config
    urdf_file = os.path.join(pkg_path, 'description', 'description.urdf.xacro') # Change to your filename
    robot_description_content = open(urdf_file, 'r').read()
    
    robot_controllers = os.path.join(pkg_path, 'config', 'steer_bot_hardware.yaml')

    # 2. Robot State Publisher (Solves the Warning)
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 3. Controller Manager
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content}, robot_controllers],
        output='both'
    )

    # 4. Spawners (These load your specific controllers)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    steering_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        steering_spawner
    ])