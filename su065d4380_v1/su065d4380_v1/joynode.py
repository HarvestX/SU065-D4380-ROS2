import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy


import serial
import struct
from operator import xor


class joyconnode(Node):
    def __init__(self):
        super().__init__('Joycon_node')

        self.cmdvelpub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joysub = self.create_subscription(
            Joy,
            '/joy',
            self.joynode_callback,
            10
        )

        self.vel_msg = Twist()

    def joynode_callback(self, msg):
        self.vel_msg.linear.x = msg.axes[1] * 0.2
        self.vel_msg.angular.z = msg.axes[0] * 0.2

        rclpy.logging._root_logger.info(str(self.vel_msg.linear.x))
        rclpy.logging._root_logger.info(str(self.vel_msg.angular.z))

        self.cmdvelpub.publish(self.vel_msg)


def main(args=None):
    rclpy.init(args=args)
    joynode = joyconnode()
    rclpy.spin(joynode)

    joynode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
