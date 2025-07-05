import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Keyboard teleop node
        Node(
            package='rover',
            executable='keyboard_teleop.py',
            name='turtlebot_teleop_keyboard',
            output='screen',
            parameters=[{
                'scale_linear': 0.5,
                'scale_angular': 1.5,
            }],
            remappings=[
                ('turtlebot_teleop_keyboard/cmd_vel', 'robot_base_velocity_controller/cmd_vel'),
            ],
        ),
    ]) 