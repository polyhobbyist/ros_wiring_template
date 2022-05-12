from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_wiring_template',
            executable='ros_wiring_template',
            name='ros_wiring_template',
            output='screen'),
    ])