from launch import LaunchDescription
from launch_ros.actions import Node

from glob import glob

from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package="teleop_twist_keyboard",
        #     executable="teleop_twist_keyboard",
        #     parameters=[{"cmd_vel":"/comnode/cmd_vel"}],
        #     output='screen',
        # ),
        Node(
            package="ROS2_SU065_D4380_driver",
            executable="rs422node",
            output='screen',
            # prefix="xterm -e",
        ),
        Node(
            package="ROS2_SU065_D4380_driver",
            executable="robocon",
            output='screen',
            prefix="xterm -e",
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            prefix="xterm -e",
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
        ),
    ])