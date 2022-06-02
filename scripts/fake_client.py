#!/usr/bin/env python3

import serial


def main():
    port_name = '/dev/ttyUSB-FT232R-Mock'
    port_handler = serial.Serial(
        port=port_name,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        bytesize=serial.EIGHTBITS,
        stopbits=serial.STOPBITS_ONE,
        timeout=None,
    )

    buf = bytes()
    while(1):
        if port_handler.inWaiting():
            break
    recv = port_handler.read(port_handler.inWaiting())
    print(recv)
    send = '$8C06\r'
    port_handler.write(send.encode('ascii'))
    print(buf)


if __name__ == '__main__':
    main()
