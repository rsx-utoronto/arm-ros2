import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('use_sim_time')
    extra_gazebo_args = LaunchConfiguration('extra_gazebo_args')
    gui = LaunchConfiguration('gui')
    recording = LaunchConfiguration('recording')
    headless = LaunchConfiguration('headless')
    debug = LaunchConfiguration('debug')
    physics = LaunchConfiguration('physics')
    verbose = LaunchConfiguration('verbose')
    output = LaunchConfiguration('output')
    world_name = LaunchConfiguration('world_name')
    respawn_gazebo = LaunchConfiguration('respawn_gazebo')
    use_clock_frequency = LaunchConfiguration('use_clock_frequency')
    pub_clock_frequency = LaunchConfiguration('pub_clock_frequency')
    enable_ros_network = LaunchConfiguration('enable_ros_network')
    server_required = LaunchConfiguration('server_required')
    gui_required = LaunchConfiguration('gui_required')

    # Set command arguments based on conditions
    command_arg1 = PythonExpression([
        "'-u' if ", paused, " == 'true' else ''"
    ])
    command_arg2 = PythonExpression([
        "'-r' if ", recording, " == 'true' else ''"
    ])
    command_arg3 = PythonExpression([
        "'--verbose' if ", verbose, " == 'true' else ''"
    ])
    script_type = PythonExpression([
        "'debug' if ", debug, " == 'true' else 'gzserver'"
    ])

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('extra_gazebo_args', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('recording', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('physics', default_value='ode'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('output', default_value='screen'),
        DeclareLaunchArgument('world_name', default_value='worlds/atacama_desert.world'),
        DeclareLaunchArgument('respawn_gazebo', default_value='false'),
        DeclareLaunchArgument('use_clock_frequency', default_value='false'),
        DeclareLaunchArgument('pub_clock_frequency', default_value='100'),
        DeclareLaunchArgument('enable_ros_network', default_value='true'),
        DeclareLaunchArgument('server_required', default_value='false'),
        DeclareLaunchArgument('gui_required', default_value='false'),

        # Set use_sim_time parameter
        Node(
            package='ros2param',
            executable='ros2param',
            name='set_sim_time',
            arguments=['set', '/use_sim_time', use_sim_time],
            output='screen',
        ),

        # Set gazebo/pub_clock_frequency if use_clock_frequency is true
        Node(
            package='ros2param',
            executable='ros2param',
            name='set_pub_clock_frequency',
            arguments=['set', 'gazebo', 'pub_clock_frequency', pub_clock_frequency],
            output='screen',
            condition=IfCondition(use_clock_frequency),
        ),

        # Set gazebo/enable_ros_network
        Node(
            package='ros2param',
            executable='ros2param',
            name='set_enable_ros_network',
            arguments=['set', 'gazebo', 'enable_ros_network', enable_ros_network],
            output='screen',
        ),

        # Start Gazebo server
        Node(
            package='gazebo_ros',
            executable=script_type,
            name='gazebo',
            output=output,
            arguments=[
                command_arg1,
                command_arg2,
                command_arg3,
                '-e', physics,
                extra_gazebo_args,
                world_name
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            respawn=respawn_gazebo,
        ),

        # Start Gazebo client if gui is true
        Node(
            package='gazebo_ros',
            executable='gzclient',
            name='gazebo_gui',
            output=output,
            arguments=[command_arg3],
            condition=IfCondition(gui),
        ),
    ]) 