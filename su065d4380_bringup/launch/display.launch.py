# Copyright 2023 HarvestX Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Dict
from typing import List
from typing import Tuple

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def load_robot_description(
        package: str,
        filename: str,
        xacro_options: List[Tuple] = []) -> Dict:
    """Load robot description."""
    filepath = get_package_share_path(package) / 'urdf' / filename
    if 'xacro' in filename:
        params = []
        if xacro_options:
            for xacro_option in xacro_options:
                params.append(' {}:='.format(
                    xacro_option[0]))
                params.append(xacro_option[1])
        command = [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            str(filepath)]
        robot_description_content = Command(command=(command + params))
    else:
        try:
            with open(str(filepath), 'r') as file:
                robot_description_content = file
        except EnvironmentError:
            exit(1)

    return {'robot_description': robot_description_content}


def load_rviz2(package: str,
               filename: str):
    """Load rviz configuration file."""
    return str(
        get_package_share_path(package) /
        'rviz' / filename)


def generate_launch_description():
    """Launch rviz display."""
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            load_robot_description(
                'su065d4380_description', 'su065d4380.urdf.xacro')
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', load_rviz2('su065d4380_bringup', 'display.rviz')]
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',)

    ld = LaunchDescription()
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)
    ld.add_action(rviz_node)

    return ld
