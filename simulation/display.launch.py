import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    rover_dir = get_package_share_directory('rover')

    # Declare launch arguments
    model = LaunchConfiguration('model')

    return LaunchDescription([
        DeclareLaunchArgument('model'),

        # Set robot description parameter
        Node(
            package='ros2param',
            executable='ros2param',
            name='set_robot_description',
            arguments=['set', 'robot_description', os.path.join(rover_dir, 'urdf', 'Rover.urdf')],
            output='screen',
        ),

        # Joint state publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(rover_dir, 'urdf.rviz')],
            output='screen',
        ),
    ]) 