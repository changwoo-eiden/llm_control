#!/usr/bin/env python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 기본 설정
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled")
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled")
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled")
    odometry_source = LaunchConfiguration("odometry_source")
    robot_namespace = LaunchConfiguration("robot_namespace")

    # xacro 경로 지정
    bcr_bot_path = FindPackageShare('bcr_bot')
    xacro_path = PathJoinSubstitution([bcr_bot_path, 'urdf', 'bcr_bot.xacro'])

    # robot_state_publisher 노드
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command([
                    'xacro', ' ',  # 공백 명시
                    xacro_path, ' ',
                    'camera_enabled:=', camera_enabled, ' ',
                    'stereo_camera_enabled:=', stereo_camera_enabled, ' ',
                    'two_d_lidar_enabled:=', two_d_lidar_enabled, ' ',
                    'sim_gazebo:=true', ' ',
                    'odometry_source:=', odometry_source, ' ',
                    'robot_namespace:=', robot_namespace
                ]),
                value_type=str
            )
        }],
        remappings=[
            ('/joint_states', [robot_namespace, '/joint_states']),
        ]
    )

    # Gazebo에 로봇 스폰
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-entity', [robot_namespace, '_robot'],
            '-z', '0.28',
            '-x', position_x,
            '-y', position_y,
            '-Y', orientation_yaw
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_enabled', default_value='true'),
        DeclareLaunchArgument('stereo_camera_enabled', default_value='false'),
        DeclareLaunchArgument('two_d_lidar_enabled', default_value='true'),
        DeclareLaunchArgument('position_x', default_value='0.0'),
        DeclareLaunchArgument('position_y', default_value='0.0'),
        DeclareLaunchArgument('orientation_yaw', default_value='0.0'),
        DeclareLaunchArgument('odometry_source', default_value='world'),
        DeclareLaunchArgument('robot_namespace', default_value='bcr_bot'),
        robot_state_publisher,
        spawn_entity
    ])
