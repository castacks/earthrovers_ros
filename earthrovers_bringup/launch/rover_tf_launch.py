import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch.actions
import launch_ros.descriptions
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():

    default_model_path = os.path.join(get_package_share_directory('earthrovers_description'), 'src/description/frodobot.urdf')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': launch_ros.descriptions.ParameterValue(Command(['xacro ',default_model_path]), value_type=str)}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': launch_ros.descriptions.ParameterValue(Command(['xacro ',default_model_path]), value_type=str)}]
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        robot_state_publisher,
        joint_state_publisher_node,
    ])