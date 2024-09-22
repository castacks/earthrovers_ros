from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='earthrovers_ros',
                executable='base',
                name='base',
                arguments=['--ros-args', '--log-level', "debug"]
            ),
            Node(
                package='earthrovers_ros',
                executable='camera',
                name='camera',
            )
        ]
    )