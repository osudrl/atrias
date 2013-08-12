#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
from time import time
import struct

serialPort = '/dev/ttyUSB0'
baudrate = '460800'
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


    loopCount   = 0
    packetCount = 0
    l = ['']
    p = ''
    dt = 0.001   # Delta time in seconds

    # Absolute angle
    x = 0.0
    y = 0.0
    z = 0.0

    # Change in angle
    dx = 0.0
    dy = 0.0
    dz = 0.0

    nextLoopTime = time()   # Don't use clock(), because it's inaccurate.
    while True:
        if time() >= nextLoopTime:
            nextLoopTime += dt

            d = ser.readline()   # Raw data
            l = l[:-1] + (l[-1] + d).split("\r\n")   # Get packets. The last element in l may not necessarily be a whole packet.

            # If we have at least one whole packet
            if len(l) > 1:
                p = l[0].split(' ')[1];
                l = l[1:]   # Pop off packet that was just read.

                try:
                    dx = struct.unpack('>f',   p[:8].decode('hex'))[0] * 180/3.1415926535 * dt    # Interpret as big-endian float.
                    dy = struct.unpack('>f', p[8:16].decode('hex'))[0] * 180/3.1415926535 * dt    # Interpret as big-endian float.
                    dz = struct.unpack('>f',  p[16:].decode('hex'))[0] * 180/3.1415926535 * dt    # Interpret as big-endian float.
                except:
                    pass

                x += dx   # Integrate
                y += dy   # Integrate
                z += dz   # Integrate

                packetCount = (packetCount+1) % 1000

            loopCount = (loopCount+1) % 1000

            # Print out somewhat slowly.
            if loopCount % 10 == 0:
                print p, "%4i %4i Rate (rad): %10f %10f %10f   Delta angle (deg): %10f %10f %10f" % (loopCount, packetCount, dx, dy, dz, x, y, z)


# vim: expandtab

