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
    tf_map_scanmatch_transform_frame_name = LaunchConfiguration('tf_map_scanmatch_transform_frame_name')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')

    return LaunchDescription([
        DeclareLaunchArgument('tf_map_scanmatch_transform_frame_name', default_value='scanmatcher_frame'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),

        # Set use_sim_time parameter
        Node(
            package='ros2param',
            executable='ros2param',
            name='set_sim_time',
            arguments=['set', '/use_sim_time', 'true'],
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_hector_slam',
            arguments=['-d', os.path.join(rover_dir, 'rover', 'simulation', 'config', 'hector_slam_occupancy_grid_sim.rviz')],
            output='screen',
        ),

        # Hector Mapping node
        Node(
            package='hector_mapping',
            executable='hector_mapping',
            name='hector_mapping',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'base_frame': base_frame,
                'odom_frame': odom_frame,
            }, os.path.join(rover_dir, 'rover', 'simulation', 'config', 'hector_slam_occupancy_grid_params_sim.yaml')],
        ),

        # Static transform publisher for odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
        ),
    ]) 