from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # Arm_Controller node
        Node(
            package='arm-ros2',
            executable='arm_controller.py',
            name='Arm_Controller',
            output='screen'
        ),

        # Arm_Manul node
        Node(
            package='arm-ros2',
            executable='manual.py',
            name='Arm_Manual',
            output='screen'
        ),

        # Arm_Safety node
        Node(
            package='arm-ros2',
            executable='safety.py',
            name='Arm_Safety',
            output='screen'
        )
    ])
