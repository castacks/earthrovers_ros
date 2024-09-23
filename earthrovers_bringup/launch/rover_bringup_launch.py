import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch.actions
import launch_ros.descriptions
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():

    rviz_config_path = os.path.join(get_package_share_directory('earthrovers_viz'), 'rover_config.rviz')
    default_model_path = os.path.join(get_package_share_directory('earthrovers_description'), 'src/description/frodobot.urdf')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': launch_ros.descriptions.ParameterValue(Command(['xacro ',default_model_path]), value_type=str)}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    earthrovers_base_node = Node(
        package='earthrovers_base',
        executable='base',
        name='base',
        arguments=['--ros-args', '--log-level', "debug"]
    )

    earthrovers_vision_node = Node(
        package='earthrovers_vision',
        executable='cameras',
        name='cameras',
    )

    earthrovers_nav_node = Node(
        package='earthrovers_navigation',
        executable='nav',
        name='nav',
    )

    earthrovers_waypoint_receiver_node = Node(
        package='earthrovers_navigation',
        executable='waypoint_receiver',
        name='waypoint_receiver',
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        robot_state_publisher,
        rviz_node,
        earthrovers_base_node,
        earthrovers_vision_node,
        earthrovers_nav_node,
        earthrovers_waypoint_receiver_node,
    ])