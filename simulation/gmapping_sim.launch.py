import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    rover_dir = get_package_share_directory('rover')

    # Declare launch arguments
    scan_topic = LaunchConfiguration('scan_topic')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')

    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='rrbot/laser/scan'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),

        # SLAM Gmapping node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_gmapping',
            output='screen',
            parameters=[os.path.join(rover_dir, 'rover', 'simulation', 'config', 'gmapping_params.yaml')],
            remappings=[
                ('scan', scan_topic),
            ],
        ),

        # Static transform publisher for laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_transform',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'hokuyo_link'],
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(rover_dir, 'rover', 'simulation', 'config', 'gmapping_config.rviz')],
            output='screen',
        ),
    ]) 