#!/usr/bin/env bash

CURRENT_USER=$USER
CURRENT_SERIAL_0="/dev/ttyUSB-FT232R-Mock"
CURRENT_SERIAL_1="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB0PCR2T-if00-port0"

sudo socat -d -d pty,raw,echo=0,link=$CURRENT_SERIAL_0 pty,raw,echo=0,link=$CURRENT_SERIAL_1 &
sleep 1; sudo chown $CURRENT_USER $CURRENT_SERIAL_0 &
sleep 1; sudo chown $CURRENT_USER $CURRENT_SERIAL_1 &
wait

unset CURRENT_USER CURRENT_SERIAL_0 CURRENT_SERIAL_1
