import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


import serial
import struct


class wheel:
    speed = 0
    enc = 0
    vel_cmd_f = 0.0
    vel_cmd_int = 0

    def __init__(self, drvid):
        self.id = drvid


class AGV:
    leftwheel = wheel(0xa2)
    rightwheel = wheel(0xa1)
    drv_status = 0
    status_id = 0xa3
    drv_error = 0
    drv_error_log = 0
    enc_id = 0xa4
    drv_bat_str = ""
    drv_bat = 0
    error_reset = 0
    break_release = 0
    bat_id = 0xa5
    comarray = [0, 0, 0, 0, 0, 0]
    response = 1
    use_response = 1
    driver_in = 0

    # bit0 : 運転開始 OFF(0)/ON(1) ※入力信号 IN1 と and 条件でモータ回転
    # bit1 : 予備
    # bit3 : ブレーキ強制解放(1) ※入力信号 IN6 と or 条件でブレーキ強制解放
    # bit4 : 予備
    # bit5 : エラーリセット ※入力信号 IN5 と or 条件でエラーリセット
    read_buf = b""

    ser = {}


# while True:
#       if ser.in_waiting > 0:
#             recv_data = ser.read(8)
#             print(type(recv_data))
#             print(a)

class AGVcontrolNode(Node):
    def __init__(self):
        self.without_ser = 0

        super().__init__('Agv_controlnode')

        self.wheelDataPublisher = self.create_publisher(
            Int32MultiArray, '/wheel_read_vel', 10)
        self.WheelCmdSubscriber = self.create_subscription(
            Int32MultiArray,
            '/wheel_order_vel',
            self.wheelcmdvell_callback,
            10
        )

        sendtimer_ms = 0.015  # seconds

        recievetimer_ms = 0.04  # seconds

        publishtimer_ms = 0.04  # seconds

        self.pubdata = Int32MultiArray(data=[0, 0, 0, 0, 0])

        self.cnt = 0

        self.agv = AGV()

        if self.without_ser != 1:
            self.agv.ser = serial.Serial(
                port="/dev/ttyUSB0",
                baudrate=115200,
                parity=serial.PARITY_NONE,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                timeout=None,
                #xonxoff = 0,
                #rtscts = 0,
            )

            self.recievetimer = self.create_timer(
                recievetimer_ms, self.datarecieve_callback)
            self.sendtimer = self.create_timer(
                sendtimer_ms, self.datasend_callback)

        self.datapublishtimer = self.create_timer(
            publishtimer_ms, self.datapublish_callback)

    def datasend_callback(self):
        self.agv.rightwheel.vel_cmd_int = int(self.agv.rightwheel.vel_cmd_f)
        self.agv.leftwheel.vel_cmd_int = int(self.agv.leftwheel.vel_cmd_f)

        if self.agv.error_reset == 0 and self.agv.drv_error > 0:
            self.agv.drv_error_log = self.agv.drv_error
            self.agv.error_reset = 1
            self.agv.drv_in = 0
            self.agv.comarray[3] = 0

        if self.agv.error_reset == 1:
            self.agv.comarray[3] = 1
            self.agv.error_reset = 0
        else:
            self.agv.comarray[3] = 0

        if self.agv.break_release == 1:

            self.agv.comarray[5] = 1
            self.agv.break_release = 0
        else:
            self.agv.comarray[5] = 0

        self.tmpsenddata1 = self.make_senddata(
            self.agv.comarray, self.agv.rightwheel.vel_cmd_int, self.agv.leftwheel.vel_cmd_int)

        self.agv.ser.write(self.tmpsenddata1)
        self.agv.response = 0

    def datarecieve_callback(self):
        self.agv.read_buf += self.agv.ser.read(self.agv.ser.inWaiting())

        for segment in self.agv.read_buf.split(b'\r')[:-1]:
            self.recv_data(segment)
        self.agv.read_buf = self.agv.read_buf.split(b'\r')[-1]

    def datapublish_callback(self):

        self.pubdata.data[0] = self.agv.rightwheel.speed
        self.pubdata.data[1] = self.agv.leftwheel.speed
        self.pubdata.data[2] = self.agv.drv_error
        self.pubdata.data[3] = self.agv.drv_status
        self.pubdata.data[4] = self.agv.drv_bat

        if self.without_ser == 1:
            self.pubdata.data[0] = self.agv.rightwheel.vel_cmd_int
            self.pubdata.data[1] = self.agv.leftwheel.vel_cmd_int

        # rclpy.logging._root_logger.info(str(self.pubdata.data))
        self.wheelDataPublisher.publish(self.pubdata)

    def wheelcmdvell_callback(self, msg):
        self.agv.rightwheel.vel_cmd_f = msg.data[0]
        self.agv.leftwheel.vel_cmd_f = msg.data[1]
        # rclpy.logging._root_logger.info(str(self.agv.drv_bat))
        # rclpy.logging._root_logger.info(str(self.agv.rightwheel.vel_cmd_f))
        # rclpy.logging._root_logger.info(str(self.agv.leftwheel.vel_cmd_f))
        # rclpy.logging._root_logger.info(str(self.agv.comarray))
        # rclpy.logging._root_logger.info(str(self.agv.drv_error))
        # rclpy.logging._root_logger.info(str(self.agv.drv_error_log))
        self.agv.rightwheel.vel_cmd_int = int(self.agv.rightwheel.vel_cmd_f)
        self.agv.leftwheel.vel_cmd_int = int(self.agv.leftwheel.vel_cmd_f)

        if self.agv.driver_in == 0:
            if self.agv.rightwheel.vel_cmd_int != 0 or self.agv.rightwheel.vel_cmd_int != 0:
                self.agv.driver_in = 1
                self.agv.comarray[0] = 1
        # tmpsenddata1=self.make_senddata(self.agv.comarray,self.agv.rightwheel.vel_cmd_int,self.agv.leftwheel.vel_cmd_int)
        # rclpy.logging._root_logger.info(str(self.tmpsenddata1))

    def tohex(self, val, nbits):
        return hex((val+(1 << nbits)) % (1 << nbits))

    def make_senddata(self, comarray, rightvel, leftvel):
        combyte = 0
        for i in range(len(comarray)):
            # print(combyte)
            combyte = combyte + (comarray[i] << i)
        # print(bin(combyte))

        startid = 0x24
        comid = 0x8c
        yobiid = 0x00
        end = 0x0d

        testdata = struct.pack('=BBBxhh', startid, comid,
                               combyte, rightvel, leftvel)

        testdata2 = struct.unpack('>bbbbbbbb', testdata)

        tmpnum = 0

        senddata = struct.pack('>BBBxhhbB', startid, comid,
                               combyte, rightvel, leftvel, tmpnum, end)

        # print(senddata)

        ads = struct.pack('>2s', (format(0, '02X').encode()))
        # print(ads)
        if rightvel < 0:
            rightvel = rightvel & 0xFFFF

        if leftvel < 0:
            leftvel = leftvel & 0xFFFF

        senddata = struct.pack('>B2s2s2s4s4s2sB', startid, format(comid, '02X').encode(), format(combyte, '02X').encode(), format(
            yobiid, '04X').encode(), format(rightvel, '04X').encode(), format(leftvel, '04X').encode(), format(tmpnum, '02X').encode(), end)

        checkdata = struct.unpack('18b', senddata)
        checksum = self.return_xor_bytes(checkdata, 0, 14)
        tmpsumbyte = format(checksum, '02X').encode()
        senddata = bytearray(senddata)
        senddata[-3] = tmpsumbyte[0]
        senddata[-2] = tmpsumbyte[1]

        return senddata

    def read_param_data(self, paramid):

        startid = 0x24
        comid = 0x8c
        yobiid = 0x00
        end = 0x0d
        tmpnum = 0x00

        write_command = 0x57
        read_command = 0x52

        command_offset = 0x1e
        write_data = 0x00

        if (paramid - command_offset in [0, 1, 2, 3, 7, 8]) == 0:
            rclpy.logging._root_logger.info("comid error")
            return

        senddata = struct.pack('>B2sB4s2sB', startid, format(0, '02X').encode(
        ), read_command, format(paramid, '04X').encode(), format(tmpnum, '02X').encode(), end)
        checkdata = struct.unpack('11b', senddata)
        checksum = self.return_xor_bytes(checkdata, 0, 7)
        tmpsumbyte = format(checksum, '02X').encode()
        senddata = bytearray(senddata)
        senddata[-3] = tmpsumbyte[0]
        senddata[-2] = tmpsumbyte[1]

        return senddata

    def write_param_data(self, paramid, param):

        startid = 0x24
        comid = 0x8c
        yobiid = 0x00
        end = 0x0d
        tmpnum = 0x00

        write_command = 0x57
        read_command = 0x52

        command_offset = 0x1e
        write_data = 0x00

        if (paramid - command_offset in [0, 1, 2, 3, 7, 8]) == 0:
            rclpy.logging._root_logger.info("comid error")
            return

        senddata = struct.pack('>B2sB4s4s2sB', startid, format(0, '02X').encode(), read_command, format(
            paramid, '04X').encode(), format(param, '04X').encode(), format(tmpnum, '02X').encode(), end)
        checkdata = struct.unpack('15b', senddata)
        checksum = self.return_xor_bytes(checkdata, 0, 11)
        tmpsumbyte = format(checksum, '02X').encode()
        senddata = bytearray(senddata)
        senddata[-3] = tmpsumbyte[0]
        senddata[-2] = tmpsumbyte[1]

        return senddata

    def recv_data(self, readdata):
        # rclpy.logging._root_logger.info(str(readdata))
        if len(readdata) == 13:

            checkdata = struct.unpack('13b', readdata)
            checksum = self.return_xor_bytes(checkdata, 0, 10)

            tmpdata = struct.unpack('c2s2s2s2s2s2s', readdata)

            readdata = bytearray()

            readdata = readdata+bytearray(tmpdata[0])

            for aj in range(1, 7):
                readdata = readdata + \
                    bytearray(int(tmpdata[aj], 16).to_bytes(1, 'big'))

            # readdata=readdata+bytearray(tmpdata[7])

            if readdata[6] != checksum:
                print("error!")

            # rclpy.logging._root_logger.info(str(readdata))

            if readdata[1] == self.agv.rightwheel.id:
                veldata = struct.unpack('>xBBxhB', readdata)
                self.agv.rightwheel.speed = veldata[2]
                # rclpy.logging._root_logger.info(str(veldata[2]))

            elif readdata[1] == self.agv.leftwheel.id:
                veldata = struct.unpack('>xBBxhB', readdata)
                self.agv.leftwheel.speed = veldata[2]
                # rclpy.logging._root_logger.info(str(veldata[2]))

            elif readdata[1] == self.agv.status_id:
                veldata = struct.unpack('>xBHHB', readdata)
                self.agv.drv_status = veldata[1]
                self.agv.drv_error = veldata[2]
                # rclpy.logging._root_logger.info(str(veldata))

            elif readdata[1] == self.agv.enc_id:
                veldata = struct.unpack('>xBHHb', readdata)
                self.agv.rightwheel.enc = veldata[1]
                self.agv.leftwheel.enc = veldata[2]
                # rclpy.logging._root_logger.info(str(veldata))

            elif readdata[1] == self.agv.bat_id:
                veldata = struct.unpack('>xB2s2sB', readdata)
                # print(veldata)
                self.agv.drv_bat_str = veldata[1]
                # print(agv.drv_bat_str)
                self.agv.drv_bat = int.from_bytes(self.agv.drv_bat_str, "big")
                # print(agv.drv_bat)
                # rclpy.logging._root_logger.info(str(veldata))
                # rclpy.logging._root_logger.info(str(self.agv.drv_bat))

            else:
                return

        # rclpy.logging._root_logger.info(str(readdata))

        if len(readdata) == 2:
            self.agv.response = 1
            # rclpy.logging._root_logger.info(str(readdata))
            return

    def return_xor_bytes(self, bytesarray, i, j):
        a = bytesarray[i]
        for k in range(i+1, j+1):
            a = a ^ bytesarray[k]
        # print(a)
        return a


def main(args=None):
    rclpy.init(args=args)
    agvnode = AGVcontrolNode()
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
