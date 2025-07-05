import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    rover_dir = get_package_share_directory('rover')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Load EKF parameters
        Node(
            package='ros2param',
            executable='ros2param',
            name='load_ekf_params',
            arguments=['load', '/', os.path.join(rover_dir, 'rover', 'simulation', 'config', 'ekf.yaml')],
            output='screen',
        ),

        # Set robot description parameter
        Node(
            package='ros2param',
            executable='ros2param',
            name='set_robot_description',
            arguments=['set', 'robot_description', os.path.join(rover_dir, 'rover', 'simulation', 'urdfs', 'robot_base.urdf')],
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
            name='spawn_urdf',
            arguments=['-entity', 'robot_base', '-file', os.path.join(rover_dir, 'rover', 'simulation', 'urdfs', 'robot_base.urdf')],
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

        # TF remapper (Note: tf_remap doesn't exist in ROS 2, using tf2_ros instead)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_remapper',
            arguments=['0', '0', '0', '0', '0', '0', 'hokuyo_link', 'lidar_link'],
            output='screen',
        ),

        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_broadcaster',
            arguments=['0.0', '0', '0.278189004081027', '0', '0', '0', '1', 'base_link', 'lidar_link'],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_broadcaster',
            arguments=['0.0', '0', '0.278189004081027', '0', '0', '0', '1', 'base_link', 'gps_link'],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_broadcaster',
            arguments=['0', '0', '0', '0.7071068', '0', '0', '0.7071068', 'base_link', 'imu_link'],
            output='screen',
        ),

        # EKF localization node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom_node',
            output='screen',
            parameters=[{
                'frequency': 30,
                'sensor_timeout': 0.1,
                'two_d_mode': True,
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                'transform_time_offset': 0.0,
                'odom0': '/rr_openrover_driver/odom_encoder',
                'odom0_differential': False,
                'odom0_relative': False,
                'odom0_queue_size': 10,
                'odom0_config': [False, False, False, False, False, False, True, True, False, False, False, True, False, False, False],
                'imu0': '/imu',
                'imu0_differential': False,
                'imu0_relative': True,
                'imu0_queue_size': 10,
                'imu0_remove_gravitational_acceleration': True,
                'imu0_config': [False, False, False, False, False, False, False, False, False, True, True, True, True, True, True],
                'print_diagnostics': True,
                'debug': False,
                'debug_out_file': 'debug_odom_ekf.txt',
                'process_noise_covariance': [0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0.06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0.06, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0.025, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0.025, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0.04, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.02, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.005],
                'initial_estimate_covariance': [1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                               0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                               0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0,
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9],
            }],
            remappings=[
                ('odometry/filtered', 'odom/ekf/enc_imu'),
            ],
        ),

        # Navsat transform node
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[{
                'magnetic_declination_radians': 0,
                'yaw_offset': 0,
            }],
            remappings=[
                ('/imu/data', '/imu'),
                ('/gps/fix', '/navsat/fix'),
                ('/odometry/filtered', '/odom/ekf/enc_imu'),
            ],
        ),

        # Random navsat transform node
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='random',
            output='screen',
            parameters=[{
                'odomN_config': [True, True, False, False, False, False, False, False, False, False, False, False, False, False, False],
                'odomN_differential': False,
            }],
        ),
    ]) 