#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
from time import time
import struct

medullaSerialPort = '/dev/ttyUSB0'
medullaBaudrate = '460800'
imuSerialPort = '/dev/ttyACM0'   # RS422 converter port for IMU
imuBaudrate = '921600'   # Communicating directly with IMU because I don't have packet parsing implemented on the Medulla.
newlineChar = '\n'
imuMode = 'delta'   # 'delta' or 'rate'

# Serial write.
def serWrite(myStr):
    try:
        for i in range(len(myStr)):
            ser.write(myStr[i])
    except:
        print "Unable to send data. Check connection."


if __name__ == "__main__":
    # Initialize serial connection.
    try:
        if imuMode == 'rate':
            ser = serial.Serial(medullaSerialPort, medullaBaudrate, timeout=0)
        elif imuMode == 'delta':
            ser = serial.Serial(imuSerialPort, imuBaudrate, timeout=0)
    except serial.SerialException:
        print "Unable to open specified serial port! Exiting..."
        exit(1)

    loopCount   = 0
    packetCount = 0
    errorCount  = 0   # Count number of bad packets
    l = ['']   # List of queued packets
    p = ''     # Packet to parse
    DT_IMU  = 0.002   # Delta T of IMU in seconds
    DT_READ = 0.001   # Delta T of serial read loop in seconds

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
            nextLoopTime += DT_READ

            d = ser.readline()   # Raw data
            l = l[:-1] + (l[-1] + d).split("\r\n")   # Get packets. The last element in l may not necessarily be a whole packet.

            # If we have at least one whole packet
            if len(l) > 1:
                try:
                    p = l[0]
                    l = l[1:]   # Pop off packet that was just read.

                    if len(p) == 24:
                        dx = struct.unpack('>f',   p[:8].decode('hex'))[0] * 180/3.1415926535    # Interpret as big-endian float.
                        dy = struct.unpack('>f', p[8:16].decode('hex'))[0] * 180/3.1415926535    # Interpret as big-endian float.
                        dz = struct.unpack('>f',  p[16:].decode('hex'))[0] * 180/3.1415926535    # Interpret as big-endian float.

                        if imuMode == 'rate':
                            dx *= DT_IMU
                            dy *= DT_IMU
                            dz *= DT_IMU
                    else:
                        errorCount += 1
                except:
                    pass

                x += dx   # Integrate
                y += dy   # Integrate
                z += dz   # Integrate

                packetCount = (packetCount+1) % 1000

            loopCount = (loopCount+1) % 1000

            # Print out somewhat slowly.
            if loopCount % 10 == 0:
                print p, errorCount, "%4i %4i Rate (rad): %10f %10f %10f   Delta angle (deg): %10f %10f %10f" % (loopCount, packetCount, dx, dy, dz, x, y, z)


# vim: expandtab

