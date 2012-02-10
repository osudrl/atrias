#!/usr/bin/env python

import sys
import string
import csv
import matplotlib.pyplot as plt

if (len(sys.argv) > 1):   # File specified?
    if (sys.argv[1] == "help"):
        print "Column indices:\n"
        print "  0: Time (ms)"
        print "  1: Body Angle (rad)"
        print "  2: Body Angle Velocity (r/s)"
        print "  3: Motor A Angle (rad)"
        print "  4: Motor A Angle inc"
        print "  5: Motor B Angle (rad)"
        print "  6: Motor B Angle inc"
        print "  7: Leg A Angle (rad)"
        print "  8: Leg B Angle (rad)"
        print "  9: Motor A Velocity (r/s)"
        print " 10: Motor B Velocity (r/s)"
        print " 11: Leg A Velocity (r/s)"
        print " 12: Leg B Velocity (r/s)"
        print " 13: Hip Angle (rad)"
        print " 14: Hip Angle Velocity (r/s)"
        print " 15: X Position"
        print " 16: Y Position"
        print " 17: Z Position"
        print " 18: X Velocity"
        print " 19: Y Velocity"
        print " 20: Z Velocity"
        print " 21: Motor A Current"
        print " 22: Motor B Current"
        print " 23: Toe Switch"
        print " 24: Command"
        print " 25: Thermistor A0"
        print " 26: Thermistor A1"
        print " 27: Thermistor A3"
        print " 28: Thermistor B0"
        print " 29: Thermistor B1"
        print " 30: Thermistor B2"
        print " 31: Motor A Voltage"
        print " 32: Motor B Voltage"
        print " 33: Logic A Voltage"
        print " 34: Logic B Voltage"
        print " 35: Medulla A Status"
        print " 36: Medulla B Status"
        print " 37: Time of last stance"
        print " 38: Motor A Torque"
        print " 39: Motor B Torque"
        print " 40: Motor Hip Torque\n"
    else:
        logFile = open(sys.argv[1], 'rU')
        dataList = list(csv.reader(logFile, delimiter=',', quotechar='"'))
        logFile.close()
else:
    print("File unspecified. Exiting.")

if (len(sys.argv) > 2):   # Column to plot specified?
    colNum = 0
    for item in dataList[0]:   # Run through label row.
        dataList[0][colNum] = item.strip()   # Strip whitespace.
        colNum += 1

    xLabel = "Time (ms)"
    yLabel = dataList[0][int(sys.argv[2])]   # Set X axis label to whatever the label for index sys.argv[2] is.
    myTitle = yLabel + " vs. " + xLabel

    del dataList[0]   # Delete label row.
    del dataList[-1]   # Delete potentially incomplete last row.

    for row in dataList:
        for i in [0, int(sys.argv[2])]:   # Parse only the two necessary columns
        # TODO: There might be a more elegant way to do this.
            try:
                row[i] = float(row[i].strip())   # Strip whitespace, then cast to float.
            except:
                print "Cannot convert to float!"

    xData = [x[0] for x in dataList]   # Timestamp.
    yData = [x[int(sys.argv[2])] for x in dataList]   # Specified column.
    yMin = min(yData)
    yMax = max(yData)

    plt.plot(xData, yData, linewidth=1.0)
    plt.xlabel(xLabel)
    plt.ylabel(yLabel)
    plt.title(myTitle)
    plt.axis([dataList[0][0], dataList[-1][0], yMin-(yMax-yMin)*1.05, yMax+(yMax-yMin)*1.05])   # Assuming timestamps are accurate, set X limits to first and last timestamps and Y limits to the minimum and maximum yData values.
    plt.grid(True)
    plt.show()
else:
    print("Column unspecified. Exiting.")

