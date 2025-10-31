from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_astar_planner',
            executable='planner',
            output='screen',
            parameters=[{
                'occupied_threshold': 50,
                'unknown_as_obstacle': True,
                'use_8_connected': True
            }]
        ),
        Node(
            package='simple_astar_planner',
            executable='auto_start_publisher',
            output='screen',
        ),
    ])
