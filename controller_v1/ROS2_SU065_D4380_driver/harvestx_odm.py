import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Path, Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster

import math


class Robot_controller(Node):
    def __init__(self):
        self.without_ser = 1

        self.dz = 0
        self.dr = 0  # [m/s]
        self.dx = 0  # [m/s]

        self.bat = 0
        self.state = 0
        self.error = 0

        self.encright = 0
        self.encleft = 0

        self.width = 0.5  # [横幅のm]

        self.rightvel = 0
        self.leftvel = 0

        self.timesys = 0

        #  realvel =  rpm / 60 / 9 * 150 * math.pi / 1000 [m/s]

        # rpm  = 9 * 1000 * 60 / 150 / math.pi * vel

        self.rotratio = 3600 / math.pi

        self.wheelordermsg = Int32MultiArray(data=[0, 0])

        self.odmmsg = Odometry()

        super().__init__('Agv_controlnode')

        self.wheelorderPublisher = self.create_publisher(
            Int32MultiArray, '/wheel_order_vel', 10)

        self.navodmpublisher = self.create_publisher(Odometry, '/nav_odm', 10)

        self.WheelCmdSubscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twisttovel_callback,
            10
        )

        self.WheelreadvelSubscriber = self.create_subscription(
            Int32MultiArray,
            '/wheel_read_vel',
            self.wheelreadvel_callback,
            10
        )

        sendtimer_ms = 0.05  # seconds

        recievetimer_ms = 0.02  # seconds

        publishtimer_ms = 0.02  # seconds

        self.cnt = 0

        if self.timesys == 1:
            self.sendtimer = self.create_timer(
                publishtimer_ms, self.datapub_callback)

    def twisttovel_callback(self, msg):
        self.dx = msg.linear.x
        self.dy = msg.linear.y
        self.dr = msg.angular.z

        self.rightvel = self.dx + self.dr * self.width / 2

        self.leftvel = self.dx - self.dr * self.width / 2

        # if self.rightvel > 0.1:
        #     self.rightvel = 0.1
        # elif self.rightvel < -0.1:
        #     self.rightvel = -0.1

        # if self.leftvel > 0.1:
        #     self.leftvel = 0.1
        # elif self.leftvel < -0.1:
        #     self.lefttvel = -0.1

        self.rightrot = int(self.rightvel * self.rotratio) * (-1)

        self.leftrot = int(self.leftvel * self.rotratio)

        self.wheelordermsg.data[0] = self.rightrot

        self.wheelordermsg.data[1] = self.leftrot

        if self.timesys == 0:
            self.wheelorderPublisher.publish(self.wheelordermsg)

        rclpy.logging._root_logger.info(str(self.rightvel))
        rclpy.logging._root_logger.info(str(self.leftvel))

    def wheelreadvel_callback(self, msg):
        # rclpy.logging._root_logger.info(str(msg.data))
        self.encright = msg.data[0] / self.rotratio  # rightvel
        self.encleft = msg.data[1] / self.rotratio * (-1)  # leftvel

        self.error = msg.data[2]
        self.state = msg.data[3]
        self.bat = msg.data[4]

        self.odmmsg.twist.twist.linear.x = (self.encright + self.encleft) / 2

        self.odmmsg.twist.twist.angular.z = (
            self.encright - self.encleft) / self.width

        self.odmmsg.twist.twist.linear.y = 0.0

        if self.timesys == 0:
            self.navodmpublisher.publish(self.odmmsg)

        # next
    def datapub_callback(self):
        self.navodmpublisher.publish(self.odmmsg)
        self.wheelorderPublisher.publish(self.wheelordermsg)


def main(args=None):
    rclpy.init(args=args)
    robocon = Robot_controller()
    rclpy.spin(robocon)

    robocon.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
