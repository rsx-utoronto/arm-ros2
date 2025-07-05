import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    rover_dir = get_package_share_directory('rover')

    return LaunchDescription([
        # Laserscan to pointcloud node
        Node(
            package='rover',
            executable='laserscan_to_pc.py',
            name='laserscan_to_pointcloud',
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(rover_dir, 'rover', 'simulation', 'config', 'laserscan_to_pc.rviz')],
            output='screen',
        ),
    ]) 