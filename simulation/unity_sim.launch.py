import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    topics = LaunchConfiguration('topics')

    return LaunchDescription([
        DeclareLaunchArgument('topics', default_value=''),

        # Unity translator node
        Node(
            package='rover',
            executable='unity_translation.py',
            name='unity_translator',
            output='screen',
            parameters=[{
                'topics': topics,
            }],
        ),
    ]) 