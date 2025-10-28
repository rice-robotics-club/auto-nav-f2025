import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz for visualization'
    )

    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='webcam',
        description='Camera type to use: webcam or realsense'
    )

    stream_type_arg = DeclareLaunchArgument(
        'stream_type',
        default_value='color',
        description='RealSense stream type: color, depth, infra1, or infra2'
    )

    # Get configuration file paths
    aruco_params = os.path.join(
        get_package_share_directory('aruco_detection'),
        'config',
        'aruco_parameters.yaml'
    )

    realsense_params = os.path.join(
        get_package_share_directory('aruco_detection'),
        'config',
        'realsense_camera.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory('aruco_detection'),
        'config',
        'aruco_rviz.rviz'
    )

    # Static transform: map -> camera_link
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_link']
    )

    # Webcam publisher node (only if camera_type == 'webcam')
    webcam_publisher_node = Node(
        package='aruco_detection',
        executable='webcam_publisher',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('camera_type'), "' == 'webcam'"])
        )
    )

    # RealSense camera node (only if camera_type == 'realsense')
    realsense_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        parameters=[realsense_params],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('camera_type'), "' == 'realsense'"])
        )
    )

    # RealSense publisher wrapper (only if camera_type == 'realsense')
    realsense_publisher_node = Node(
        package='aruco_detection',
        executable='realsense_publisher',
        parameters=[{'stream_type': LaunchConfiguration('stream_type')}],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('camera_type'), "' == 'realsense'"])
        )
    )

    # ArUco detection node
    aruco_node = Node(
        package='aruco_detection',
        executable='aruco_node',
        parameters=[aruco_params]
    )

    # RViz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        camera_type_arg,
        stream_type_arg,
        # Nodes
        static_transform_node,
        webcam_publisher_node,
        realsense_camera_node,
        realsense_publisher_node,
        aruco_node,
        rviz_node
    ])
