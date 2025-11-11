from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world_path = PathJoinSubstitution([
        FindPackageShare("park_worlds"),
        "worlds",
        "big_park.world"
    ])

    return LaunchDescription([
        ExecuteProcess(
            cmd=["gazebo", "--verbose", world_path],
            output="screen"
        )
    ])

