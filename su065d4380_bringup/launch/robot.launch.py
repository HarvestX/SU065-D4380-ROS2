"""Launch robot."""
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    controller_pkg = 'su065d4380_v1'
    enkf_param = str(get_package_share_path(
        'robot_localization') / 'params' / 'ekf.yaml')
    return LaunchDescription([
        Node(
            package=controller_pkg,
            executable="comnode",
            output='screen',
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
    ])
