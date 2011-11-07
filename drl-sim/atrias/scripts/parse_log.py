#!/usr/bin/env python

import sys
import string
import csv
import matplotlib.pyplot as plt

if (len(sys.argv) > 1):   # File specified?
    logFile = open(sys.argv[1], 'rU')
    dataList = list(csv.reader(logFile, delimiter=',', quotechar='"'))
    logFile.close()
else:
    print("File unspecified. Exiting.")

if (len(sys.argv) > 2):   # Column to plot specified?
    ylabel = dataList[0][int(sys.argv[2])]
    del dataList[0]   # Delete label row.
    del dataList[-1]   # Delete potentially incomplete last row.
    for row in dataList:
        colNum = 0
        for col in row:
            row[colNum] = float(col.strip())
            colNum += 1

    xdata = [x[0] for x in dataList]   # Timestamp
    ydata = [x[int(sys.argv[2])] for x in dataList]

    plt.plot(xdata, ydata, linewidth=1.0)
    plt.xlabel('Time (ms)')
    plt.ylabel('Motor Angle B')
    plt.title('Title!')
    plt.axis([dataList[0][0], dataList[-1][0], -1, 1])
    plt.grid(True)
    plt.show()
else:
    print("Column unspecified. Exiting.")

