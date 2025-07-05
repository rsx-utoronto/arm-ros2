import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    rover_dir = get_package_share_directory('rover')

    # Declare launch arguments
    no_static_map = LaunchConfiguration('no_static_map')
    command_velocity_topic = LaunchConfiguration('command_velocity_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    use_map_topic = LaunchConfiguration('use_map_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    map_topic = LaunchConfiguration('map_topic')

    return LaunchDescription([
        # Set use_sim_time parameter
        Node(
            package='ros2param',
            executable='ros2param',
            name='set_sim_time',
            arguments=['set', '/use_sim_time', 'true'],
            output='screen',
        ),

        # Declare launch arguments
        DeclareLaunchArgument('no_static_map', default_value='false'),
        DeclareLaunchArgument('command_velocity_topic', default_value='/robot_base_velocity_controller/cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='/robot_base_velocity_controller/odom'),
        DeclareLaunchArgument('use_map_topic', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='rrbot/laser/scan'),
        DeclareLaunchArgument('map_topic', default_value='map'),

        # Include Hector SLAM launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rover_dir, '/rover/simulation/launch', '/hector_slam_occupancy_grid_sim.launch.py']),
        ),

        # Move base node
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='move_base',
            output='screen',
            parameters=[
                os.path.join(rover_dir, 'rover', 'simulation', 'config', 'costmap_common.yaml'),
                os.path.join(rover_dir, 'rover', 'simulation', 'config', 'local_costmap.yaml'),
                os.path.join(rover_dir, 'rover', 'simulation', 'config', 'global_costmap.yaml'),
                os.path.join(rover_dir, 'rover', 'simulation', 'config', 'move_base.yaml'),
                {
                    'base_global_planner': 'navfn/NavfnROS',
                    'base_local_planner': 'dwa_local_planner/DWAPlannerROS',
                    'controller_frequency': 5.0,
                    'controller_patience': 15.0,
                }
            ],
            remappings=[
                ('cmd_vel', command_velocity_topic),
                ('odom', odom_topic),
            ],
        ),

        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            arguments=[os.path.join(rover_dir, 'rover', 'simulation', 'maps', 'white.yaml')],
            output='screen',
            parameters=[{
                'frame_id': 'map',
            }],
        ),

        # AMCL node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                os.path.join(rover_dir, 'rover', 'simulation', 'config', 'amcl_params.yaml'),
                {
                    'initial_pose_x': 0,
                    'initial_pose_y': 0,
                    'initial_pose_a': 0,
                }
            ],
            remappings=[
                ('scan', scan_topic),
            ],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_move_base',
            arguments=['-d', os.path.join(rover_dir, 'rover', 'simulation', 'config', 'move_base.rviz')],
            output='screen',
        ),
    ]) 