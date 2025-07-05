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
            arguments=['set', 'robot_description', os.path.join(rover_dir, 'rover', 'simulation', 'urdfs', 'new_urdf', 'urdf', 'URDF.xacro')],
            output='screen',
        ),

        # Include empty world launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_ros_dir, '/launch', '/empty_world.launch.py']),
        ),

        # Spawn URDF model
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_model',
            arguments=['-entity', 'Rover', '-file', os.path.join(rover_dir, 'rover', 'simulation', 'urdfs', 'new_urdf', 'urdf', 'URDF.xacro'), '-z', '5'],
            output='screen',
        ),

        # Load controller parameters
        Node(
            package='ros2param',
            executable='ros2param',
            name='load_controller_params',
            arguments=['load', '/', os.path.join(rover_dir, 'rover', 'simulation', 'urdfs', 'new_urdf', 'launch', 'controller.yaml')],
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
    ]) 