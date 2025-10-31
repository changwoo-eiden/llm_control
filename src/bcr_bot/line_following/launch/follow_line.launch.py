from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_following',
            executable='follow_yellow_line',
            name='follow_yellow_line',
            output='screen'
        )
    ])

