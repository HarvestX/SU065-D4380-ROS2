import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import String


import serial
import struct
from operator import xor


class paramkeyNode(Node):
    def __init__(self):
        self.without_ser = 0

        super().__init__('paramkeynode')

        self.keypub=self.create_publisher(String,'/param_key',10)
        self.valuepub=self.create_publisher(Int32MultiArray,'/param_value',10)


        keyinputtimer_ms = 0.01  # seconds

        self.keyinputtimer = self.create_timer(keyinputtimer_ms,self.keyinput_callback)

    def keyinput_callback(self):
        key = input("r:read all data , w:write one parameter,, e: reset error\n")

        pubkey = String(data=key)
        self.keypub.publish(pubkey)
        rclpy.logging._root_logger.info(str(key))

        if key == "r":
            return 

        elif key =='w':
            indexkey = input("0:right total gain, 1:left total gain,2 :acc time,3:dcc time \n 4:comm timeout[0.1s],5:gensoku time\n")
            valuekey = input("value")
            indexkey = int(indexkey)
            valuekey= int(valuekey)

            if valuekey > 100 and indexkey in [0,1]:
                valuekey = 100

            if valuekey > 500 and indexkey in [2,3,5]:
                valuekey = 500
            
            if valuekey > 5 and indexkey ==4:
                valuekey = 5
            
            pubdata=Int32MultiArray(data=[indexkey,valuekey]) 

            self.valuepub.publish(pubdata)
            
        elif key =="e":
            return 
        
def main(args=None):
    rclpy.init(args=args)
    agvnode = paramkeyNode()
    rclpy.spin(agvnode)

    agvnode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# comtmparray = [1,0,0,0,0,0]

# make_senddata(comtmparray,3000,0)

# exdata = "$A10100F83028\r"

# exbatdata = "$A512D3000028\r"

# exbatdata = exbatdata.encode()

# aas=exdata.encode()

# print(aas)

# recv_data(aas,agv_aa)

# recv_data(exbatdata,agv_aa)




