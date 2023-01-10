#!/usr/bin/env python3
# Copyright 2022 HarvestX Inc.
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

import serial


# Need to append crc and \r
driver_info_packet_base = [
    '$A10100F830',
    '$A200000000',
    '$A300000000',
    '$A40100F830',
    '$A512D30000', ]


def calcCRC(packet: str) -> str:
    crc = 0
    for p in packet.encode():
        crc = crc ^ p
    return format(crc, '02X')


def create_driver_info_packet(key: int) -> str:
    if(key > len(driver_info_packet_base)):
        raise RuntimeError('Invalid key given')

    target_data = driver_info_packet_base[key]
    crc = calcCRC(target_data)
    return target_data + crc + '\r'


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

    while(1):
        if port_handler.inWaiting():
            break
    recv = port_handler.read(port_handler.inWaiting())
    print(recv)
    response = '$8C06\r'
    port_handler.write(response.encode('ascii'))

    for i in range(len(driver_info_packet_base)):
        packet = create_driver_info_packet(i)
        port_handler.write(packet.encode('ascii'))
        print(packet)


if __name__ == '__main__':
    main()
