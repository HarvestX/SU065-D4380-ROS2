#!/usr/bin/env bash

CURRENT_USER=$USER
CURRENT_SERIAL_0="/dev/ttyUSB-FT232R-Mock"
CURRENT_SERIAL_1="/dev/ttyUSB-SU065-D4380"

sudo mkdir -p /dev/serial/by-id
sudo socat -d -d pty,raw,echo=0,link=$CURRENT_SERIAL_0 pty,raw,echo=0,link=$CURRENT_SERIAL_1 &
sleep 1

sudo chown $CURRENT_USER $CURRENT_SERIAL_0 &
sleep 1

sudo chown $CURRENT_USER $CURRENT_SERIAL_1 &
wait

unset CURRENT_USER CURRENT_SERIAL_0 CURRENT_SERIAL_1
