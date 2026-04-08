from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. Resize Node for Left Image
    resize_left = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        name='resize_left',
        parameters=[{
            'output_width': 960,
            'output_height': 576,
            'keep_aspect_ratio': False,
            'encoding_desired': 'bgr8',
        }],
        remappings=[
            ('image', '/camera/camera/infra1/image_rect_raw'),
            ('camera_info', '/camera/camera/infra1/camera_info'),
            ('resize/image', '/infra1/resized'),
            ('resize/camera_info', '/infra1/camera_info_resized')
        ]
    )

    # 2. Resize Node for Right Image
    resize_right = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        name='resize_right',
        parameters=[{
            'output_width': 960,
            'output_height': 576,
            'keep_aspect_ratio': False,
            'encoding_desired': 'bgr8',
        }],
        remappings=[
            ('image', '/camera/camera/infra2/image_rect_raw'),
            ('camera_info', '/camera/camera/infra2/camera_info'),
            ('resize/image', '/infra2/resized'),
            ('resize/camera_info', '/infra2/camera_info_resized')
        ]
    )

    # 3. Your ESS Node (Updated remappings to use resized images)
    ess_node = ComposableNode(
        package='isaac_ros_ess',  # Keep this as 'isaac_ros_ess'
        plugin='nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode', # Update this namespace
        name='ess_disparity_node',
        remappings=[
            ('left/image_rect', '/infra1/resized'),
            ('right/image_rect', '/infra2/resized'),
            ('left/camera_info', '/infra1/camera_info_resized'),
            ('right/camera_info', '/infra2/camera_info_resized'),
        ],
        parameters=[{
            'engine_file_path': '/workspaces/isaac_ros-dev/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/ess.engine',
            'input_width': 960,
            'input_height': 576,
            'input_image_type': 'bgr8',
        }]
    )

    container = ComposableNodeContainer(
        name='ess_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[resize_left, resize_right, ess_node],
        output='screen',
    )

    return LaunchDescription([container])