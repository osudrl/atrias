#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
from time import time
import struct
from math import pi
from kvh_vis import *   # DCM visualizer

debugType = 'medulla'   # 'direct' or 'medulla'
imuMode = 'dcm'   # 'delta', 'rate', or 'dcm'   NOTE: dcm only works with medulla.

if debugType == 'medulla':
    serialPort = '/dev/ttyUSB0'
    baudrate = '460800'
    packetSepChar = '\r\n'
    DT_IMU  = 0.002   # Delta T of IMU in seconds
elif debugType == 'direct':
    serialPort = '/dev/ttyACM0'   # RS422 converter port for IMU
    baudrate = '921600'   # Communicating directly with IMU because I don't have packet parsing implemented on the Medulla.
    packetSepChar = '\xfe\x81\xff\x55'
    DT_IMU  = 0.001   # Delta T of IMU in seconds


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
DT_READ = 0.0008   # Delta T of serial read loop in seconds
imuStatus = 'unknown'

# Absolute angle
x = 0.0
y = 0.0
z = 0.0

# Change in angle
dx = 0.0
dy = 0.0
dz = 0.0

# DCM
dcm = [[1.0, 0.0, 0.0],
       [0.0, 1.0, 0.0],
       [0.0, 0.0, 1.0]]

if imuMode == 'dcm':
    setupVisualizer()

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
                if debugType == 'medulla':
                    if imuMode == 'dcm' and len(p) == 36:
                        for i in range(3):
                            for j in range(3):
                                dcm[i][j] = struct.unpack('>f', p[i*3+j:i*3+j+4])[0] * 180/pi
                        updateDCM(dcm)
                        updateVisualizer()
                    elif len(p) == 24:
                        dx = struct.unpack('>f',   p[:8].decode('hex'))[0] * 180/pi
                        dy = struct.unpack('>f', p[8:16].decode('hex'))[0] * 180/pi
                        dz = struct.unpack('>f',  p[16:].decode('hex'))[0] * 180/pi
                elif debugType == 'direct' and len(p) == 32:
                    dx = struct.unpack('>f',   p[:4])[0] * 180/pi
                    dy = struct.unpack('>f',  p[4:8])[0] * 180/pi
                    dz = struct.unpack('>f', p[8:12])[0] * 180/pi
                    imuStatus = bin(ord(p[24]))
                else:
                    errorCount += 1

                if imuMode == 'rate':
                    dx *= DT_IMU
                    dy *= DT_IMU
                    dz *= DT_IMU

            except:
                pass   # Sometimes we'll mangle the first packet. Ignore this.

            x += dx   # Integrate
            y += dy   # Integrate
            z += dz   # Integrate

            packetCount = packetCount+1

        else:
            haveNewPacket = False

        loopCount = loopCount+1

        if haveNewPacket:
            if imuMode == 'dcm':
                print "%2i %2i %8i %8i   DCM: [%10f %10f %10f] [%10f %10f %10f] [%10f %10f %10f]" % (errorCount, len(p), loopCount, packetCount, dcm[0][0], dcm[0][1], dcm[0][2], dcm[1][0], dcm[1][1], dcm[1][2], dcm[2][0], dcm[2][1], dcm[2][2])
            else:
                print "%2i %8i %8i %10s   %5s (rad): %10f %10f %10f   angle (deg): %10f %10f %10f" % (errorCount, loopCount, packetCount, imuStatus, imuMode, dx, dy, dz, x, y, z)


# vim: expandtab

