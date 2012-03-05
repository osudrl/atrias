#!/usr/bin/env python

import sys
import string
import csv
import matplotlib.pyplot as plt

if (len(sys.argv) > 1):   # File specified?
    if (sys.argv[1] == "help"):
        print "Column indices:\n"
	print "  0: Tiime (ms)"
	print "  1: Body Angle"
	print "  2: Body Angle Velocity"
	print "  3: Body Pitch"
	print "  4: Body Pitch Velocity"
	print "  5: Motor A Angle"
	print "  6: Motor A Angle (inc)"
	print "  7: Motor B Angle"
	print "  8: Motor B Angle (inc)"
	print "  9: Leg A Angle"
	print " 10: Leg B Angle"
	print " 11: Motor A Velocity"
	print " 12: Motor Belocity"
	print " 13: Leg A Velocity"
	print " 14: Leg B Velocity"
	print " 15: Hip Angle"
	print " 16: Hip Angular Velocity"
	print " 17: X Position"
	print " 18: Y Position"
	print " 19: Z Position"
	print " 20: X Velocity"
	print " 21: Y Velocity"
	print " 22: Z Velocity"
	print " 23: Motor A Current"
	print " 24: Motor B Current"
	print " 25: Toe Switch"
	print " 26: Toe Force"
	print " 27: Command"
	print " 28: Thermistor A0"
	print " 29: Thermistor A1"
	print " 30: Thermistor A2"
	print " 31: Thermistor B0"
	print " 32: Thermistor B1"
	print " 33: Thermistor B2"
	print " 34: Motor A Voltage"
	print " 35: Motor B Voltage"
	print " 36: Logic A Voltage"
	print " 37: Logic B Voltage"
	print " 38: Medulla A Status"
	print " 39: Medulla B Status"
	print " 40: Time of Last Stance"
	print " 41: Motor A Torque"
	print " 42: Motor B Torque"
	print " 43: Motor Hip Torque"
	print " 44: Controller Phase (Stance = 0)"
	print " 45: Desired Motor Angle A"
	print " 46: Desired Motor Angle B"
	print " 47: Desired Deflection A"
	print " 48: Desired Deflection B"
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

