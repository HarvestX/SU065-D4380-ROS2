"""Launch joy con."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    this_pkg_name = 'su065d4380_v1'
    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
        ),
        Node(
            package=this_pkg_name,
            executable="joynode",
            output='screen',
        ),
    ])
