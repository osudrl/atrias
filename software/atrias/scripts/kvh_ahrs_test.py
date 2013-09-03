#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
from time import time
import struct
from math import pi

imuMode = 'delta'   # 'delta' or 'rate'

if imuMode == 'rate':
    serialPort = '/dev/ttyUSB0'
    baudrate = '460800'
    packetSepChar = '\r\n'
elif imuMode == 'delta':
    serialPort = '/dev/ttyACM0'   # RS422 converter port for IMU
    baudrate = '921600'   # Communicating directly with IMU because I don't have packet parsing implemented on the Medulla.
    packetSepChar = '\xfe\x81\xff\x55'

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
        ser = serial.Serial(serialPort, baudrate, timeout=0)
    except serial.SerialException:
        print "Unable to open specified serial port! Exiting..."
        exit(1)

    loopCount   = 0
    packetCount = 0
    errorCount  = 0   # Count number of bad packets
    l = ['']   # List of queued packets
    p = ''     # Packet to parse
    haveNewPacket = False   # Do we have a new packet (to consider printing?)
    DT_IMU  = 0.002   # Delta T of IMU in seconds
    DT_READ = 0.0008   # Delta T of serial read loop in seconds

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
            l = l[:-1] + (l[-1] + d).split(packetSepChar)   # Get packets. The last element in l may not necessarily be a whole packet.

            # If we have at least one whole packet
            if len(l) > 1:
                haveNewPacket = True
                try:
                    p = l[0]
                    l = l[1:]   # Pop off packet that was just read.

                    # Parse gyro data as big-endian floats.
                    if imuMode == 'rate':
                        if len(p) == 24:
                            dx = struct.unpack('>f',   p[:8].decode('hex'))[0] * 180/pi * DT_IMU
                            dy = struct.unpack('>f', p[8:16].decode('hex'))[0] * 180/pi * DT_IMU
                            dz = struct.unpack('>f',  p[16:].decode('hex'))[0] * 180/pi * DT_IMU
                        else:
                            errorCount += 1
                    elif imuMode == 'delta':
                        if len(p) == 32:
                            dx = struct.unpack('>f',   p[:4])[0] * 180/pi# * 720/692.66
                            dy = struct.unpack('>f',  p[4:8])[0] * 180/pi# * 720/692.66
                            dz = struct.unpack('>f', p[8:12])[0] * 180/pi# * 720/692.66
                        else:
                            errorCount += 1
                except:
                    pass   # Sometimes we'll mangle the first packet. Ignore this.

                x += dx   # Integrate
                y += dy   # Integrate
                z += dz   # Integrate

                packetCount = (packetCount+1)# % 1000

            else:
                haveNewPacket = False

            loopCount = (loopCount+1)# % 1000

            # Print out somewhat slowly.
            if haveNewPacket:#loopCount % 10 == 0:
                print "%2i %8i %8i   %5s (rad): %10f %10f %10f   angle (deg): %10f %10f %10f" % (errorCount, loopCount, packetCount, imuMode, dx, dy, dz, x, y, z)


# vim: expandtab

