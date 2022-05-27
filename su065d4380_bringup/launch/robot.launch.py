"""Launch robot."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
)
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    """Generate launch description."""
    controller_pkg = 'su065d4380_v1'
    enkf_param = str(get_package_share_path(
        'robot_localization') / 'params' / 'ekf.yaml')

    launch_args = [
        DeclareLaunchArgument(
            'dev',
            default_value=TextSubstitution(
                text='/dev/ttyUSB0'
            )
        ),
    ]

    nodes = [
        Node(
            package=controller_pkg,
            executable="comnode",
            output='screen',
            parameters=[{
                'dev': LaunchConfiguration('dev')
            }]
        ),
        Node(
            package=controller_pkg,
            executable="odomnode",
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[enkf_param]
        ),
    ]

    return LaunchDescription(launch_args + nodes)
