#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
from time import sleep

serialPort = '/dev/ttyACM0'
baudrate = '921600'
newlineChar = '\n'

# Serial write.
def serWrite(myStr):
    try:
        for i in range(len(myStr)):
            ser.write(myStr[i])
    except:
        print "Unable to send data. Check connection."


if __name__ == "__main__":
    # =========================================================================
    # Try to initialize a serial connection. If serialPort is defined, try
    # opening that. If it is not defined, loop through a range of integers
    # starting from 0 and try to connect to /dev/ttyUSBX where X is the
    # integer. In either case, process dies if serial port cannot be opened.
    #
    # TODO: This needs to be made more concise.
    # =========================================================================
    try:
        ser = serial.Serial(serialPort, baudrate, timeout=0)
    except serial.SerialException:
        print "Unable to open specified serial port! Exiting..."
        exit(1)
    except AttributeError:
        for i in range(4):
            try:
                ser = serial.Serial("/dev/ttyUSB"+str(i), baudrate, timeout=0)
                print "Opened serial port at /dev/ttyUSB%d.", i
                break
            except serial.SerialException:
                print "No serial at /dev/ttyUSB%d.", i
                if i == 3:
                    print "No serial found. Giving up!"
                    exit(1)

    while True:
        cmd = raw_input("Command to send: ")

        if cmd == 'q':
            exit(0)

        serWrite(cmd+newlineChar)

        sleep(0.02)

        if ser.inWaiting() > 0:
            print ser.read(ser.inWaiting())#.encode("hex")   # Hex encoding useful for debugging individual packets.

# vim: expandtab

