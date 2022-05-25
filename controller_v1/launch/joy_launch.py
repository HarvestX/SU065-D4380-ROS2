from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package="teleop_twist_keyboard",
        #     executable="teleop_twist_keyboard",
        #     parameters=[{"cmd_vel":"/comnode/cmd_vel"}],
        #     output='screen',
        # ),
        Node(
            package="joy",
            executable="joy_node",
        ),
        Node(
            package="ROS2_SU065_D4380_driver",
            executable="joycon",
            output='screen',
            # prefix="xterm -e",
        )
        
    ])