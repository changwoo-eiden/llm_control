#!/usr/bin/env python3

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bcr_bot_path = get_package_share_directory('bcr_bot')

    # Launch args
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    world_file = LaunchConfiguration("world_file", default=join(bcr_bot_path, 'worlds', 'small_warehouse.sdf'))

    # Include gazebo
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={
            'world': world_file,
            'gui': gui,
            'verbose': verbose,
        }.items()
    )

    # Include bcr_bot spawn launch
    spawn_bcr_bot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bcr_bot_path, "launch", "bcr_bot_gazebo_spawn.launch.py")),
        launch_arguments={
            'camera_enabled': 'true',
            'stereo_camera_enabled': 'false',
            'two_d_lidar_enabled': 'true',
            'position_x': '0.0',
            'position_y': '0.0',
            'orientation_yaw': '0.0',
            'odometry_source': 'world',
            'robot_namespace': 'bcr_bot'
        }.items()
    )

    return LaunchDescription([
        # 환경 변수 설정
        AppendEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=join(bcr_bot_path, "models")),

        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH',
            value="/usr/share/gazebo-11:" + join(bcr_bot_path, "worlds")),

        # Declare arguments
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Launch Gazebo + bot
        gazebo,
        spawn_bcr_bot_node
    ])
