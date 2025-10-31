from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_following',
            executable='follow_line',  # ✅ NOT follow_yellow_line
            name='line_follower',
            output='screen'
        )
    ])

