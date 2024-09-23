import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='earthrovers_base',
                executable='base',
                name='base',
                arguments=['--ros-args', '--log-level', "debug"]
            ),
            Node(
                package='earthrovers_vision',
                executable='cameras',
                name='cameras',
            ),
            Node(
                package='earthrovers_navigation',
                executable='nav',
                name='nav',
            ),
            Node(
                package='earthrovers_navigation',
                executable='waypoint_receiver',
                name='waypoint_receiver',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(get_package_share_directory('earthrovers_viz'), 'rover_config.rviz')]
            ),
            # NOTE: This is a temporary static transform publisher to establish
            # the odom frame. However, this eventually wouldn't be needed if the
            # node publishing odometry were also to publish a base_link --> odom
            # transform. Just haven't added that yet.
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                name='static_transform_publisher'
            ),
        ]
    )