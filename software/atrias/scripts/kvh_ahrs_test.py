#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
from time import time
import struct

serialPort = '/dev/ttyUSB0'
baudrate = '115200'
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


    loopCount = 0
    l = ['']
    p = ''
    x = 0.0
    y = 0.0
    z = 0.0

    deltaAngle = 0.0

    nextLoopTime = time()   # Don't use clock(), because it's inaccurate.
    while True:
        if time() >= nextLoopTime:
            nextLoopTime += 0.005

            d = ser.readline()   # Raw data
            l = l[:-1] + (l[-1] + d).split("\r\n")   # Get packets. The last element in l may not necessarily be a whole packet.

            # If we have at least one whole packet
            if len(l) > 1:
                p = l[0].split(' ')[1];
                l = l[1:]   # Pop off packet that was just read.

                try:
                    x = struct.unpack('>f', p[:8].decode('hex'))[0]   # Interpret as big-endian float.
                    y = struct.unpack('>f', p[8:16].decode('hex'))[0]   # Interpret as big-endian float.
                    z = struct.unpack('>f', p[16:].decode('hex'))[0]   # Interpret as big-endian float.
                except:
                    pass

            dp = deltaAngle
            deltaAngle = deltaAngle + x/200.0

            # Print out somewhat slowly.
            loopCount = (loopCount+1) % 10
            if loopCount == 0:
                print p, "Read: %10f %10f %10f   Integrated: %10f" % (x, y, z, deltaAngle)


# vim: expandtab

