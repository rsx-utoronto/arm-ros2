import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    rover_dir = get_package_share_directory('rover')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        # Set robot description parameter
        Node(
            package='ros2param',
            executable='ros2param',
            name='set_robot_description',
            arguments=['set', 'robot_description', os.path.join(rover_dir, 'rover', 'simulation', 'urdfs', 'rover.urdf')],
            output='screen',
        ),

        # Include empty world launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_ros_dir, '/launch', '/empty_world.launch.py']),
        ),

        # Static transform publisher for base_link to chassis
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'chassis'],
            output='screen',
        ),

        # Spawn URDF model
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_model',
            arguments=['-entity', 'Rover', '-file', os.path.join(rover_dir, 'rover', 'simulation', 'urdfs', 'rover.urdf')],
            output='screen',
        ),

        # Load control parameters
        Node(
            package='ros2param',
            executable='ros2param',
            name='load_control_params',
            arguments=['load', '/', os.path.join(rover_dir, 'rover', 'simulation', 'config', 'control.yaml')],
            output='screen',
        ),

        # Spawn base controller
        Node(
            package='controller_manager',
            executable='spawner',
            name='base_controller_spawner',
            arguments=['robot_base_joint_publisher', 'robot_base_velocity_controller'],
            output='screen',
        ),

        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'chassis'],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='wheel_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'front_left_wheel'],
            output='screen',
        ),
    ]) 