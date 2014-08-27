#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
from time import sleep
import sys

baudrate = '921600'
newlineChar = '\n'
in_config = False

# If serial device is unspecified, print usage guide.
if len(sys.argv) != 2:
    print("Usage:\n    " + sys.argv[0] + " <serial device>")
    exit(1)

# Open serial port.
serialPort = sys.argv[1]
ser = serial.Serial(serialPort, baudrate, timeout=0)

# Serial write.
def ser_write(myStr):
    try:
        for i in range(len(myStr)):
            ser.write(myStr[i])
    except:
        print "Unable to send data. Check connection."

# Probably don't need a main function, but this script could be a lot more than
# a basic console interface.
if __name__ == "__main__":
    while True:
        cmd = raw_input("KVH 1750 > ")

        if cmd == 'q':
            exit(0)
        if cmd == "=config,1":
            in_config = True
            ser.read(ser.inWaiting())
        if cmd == "=config,0":
            in_config = False

        ser_write(cmd+newlineChar)

        # Leave time for IMU to respond.
        sleep(0.02)

        if ser.inWaiting() > 0:
            t = ser.read(ser.inWaiting())
            if not in_config:
                t = t.encode("hex")   # Hex encoding useful for debugging individual packets.
            print(t)

# vim: expandtab

