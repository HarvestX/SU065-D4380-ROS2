"""Launch joy con."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controller_pkg = 'su065d4380_v1'
    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
        ),
        Node(
            package=controller_pkg,
            executable="joynode",
            output='screen',
        ),
    ])
