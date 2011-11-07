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
        for colNum in [0, int(sys.argv[2])]:
            try:
                row[colNum] = float(row[colNum].strip())
            except:
                pass

    xdata = [x[0] for x in dataList]   # Timestamp
    ydata = [x[int(sys.argv[2])] for x in dataList]

    plt.plot(xdata, ydata, linewidth=1.0)
    plt.xlabel('Time (ms)')
    plt.ylabel('Motor Angle B')
    plt.title('Title!')
    plt.axis([dataList[0][0], dataList[-1][0], -5, 5])
    plt.grid(True)
    plt.show()
else:
    print("Column unspecified. Exiting.")

