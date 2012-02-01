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
        print "  2: Motor A Angle (rad)"
        print "  3: Motor B Angle (rad)"
        print "  4: Leg A Angle (rad)"
        print "  5: Leg B Angle (rad)"
        print "  6: Body Angular Velocity (r/s)"
        print "  7: Motor A Velocity (r/s)"
        print "  8: Motor B Velocity (r/s)"
        print "  9: Leg A Velocity (r/s)"
        print "  0: Leg B Velocity (r/s)"
        print " 11: X Position"
        print " 12: Y Position"
        print " 13: Z Position"
        print " 14: X Velocity"
        print " 15: Y Velocity"
        print " 16: Z Velocity"
        print " 17: Horizontal Velocity"
        print " 18: Motor A Current"
        print " 19: Motor B Current"
        print " 20: Toe Switch"
        print " 21: Command"
        print " 22: Motor A Torque"
        print " 23: Motor B Torque\n"
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

