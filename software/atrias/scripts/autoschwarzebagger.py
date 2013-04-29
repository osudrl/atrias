#!/usr/bin/env python2

import sys
import os
import glob
import subprocess

if (len(sys.argv) < 3):
    print("Usage: " + sys.argv[0] + " [1] [2] [3]")
    print("    1: Directory containing bagfiles to convert")
    print("    2: Directory for fixed bagfiles and output MATLAB files")
    print("    3: Number of threads to use (default is 1)")
    exit()

# Get list of files to convert.
filesToProc = glob.glob(sys.argv[1]+"/**/*.bag")

print "Files to convert:\n"
for f in filesToProc:
    print "    " + f
print ""

# Get (sub)directories containing bagfiles.
dirList = []
walk = os.walk(sys.argv[1])
for path, dirs, files in walk:
    bagfiles = glob.glob(path+"/*.bag")
    if len(bagfiles) > 0:
        dirList.append(path)

# Create necessary subdirectories.
for path in dirList:
    subprocess.call(["mkdir", "-p", path.replace(sys.argv[1], sys.argv[2], 1)])

# Set maximum number of threads to use.
maxNumThreads = 1
if len(sys.argv) > 3:
    maxNumThreads = sys.argv[3]

# Initialize list of processes to spawn.
procList = []
nextFileToProc = 0

# Run fix_bag.py
while True:
    # Spawn off new conversion processes.
    while len(procList) < int(maxNumThreads) and nextFileToProc < len(filesToProc):
        procList.append(subprocess.Popen(["rosrun", "atrias", "fix_bag.py", filesToProc[nextFileToProc], sys.argv[2]+"/"+filesToProc[nextFileToProc]]))
        print "[" + str(nextFileToProc+1) + "/" + str(len(filesToProc)) + "] " + filesToProc[nextFileToProc]
        nextFileToProc += 1

    # Remove complete processes from list.
    procList = filter(lambda y: y.poll() == None, procList)

    if len(procList) == 0 and nextFileToProc == len(filesToProc):
        break;

# Reset things.
nextFileToProc = 0

# Run bag2mat.py
while True:
    # Spawn off new conversion processes.
    while len(procList) < int(maxNumThreads) and nextFileToProc < len(filesToProc):
        procList.append(subprocess.Popen(["rosrun", "atrias", "bag2mat.py", sys.argv[2]+"/"+filesToProc[nextFileToProc], sys.argv[2]+"/"+filesToProc[nextFileToProc].replace('.bag', '.mat')]))
        print "[" + str(nextFileToProc+1) + "/" + str(len(filesToProc)) + "] " + filesToProc[nextFileToProc]
        nextFileToProc += 1

    # Remove complete processes from list.
    procList = filter(lambda y: y.poll() == None, procList)

    if len(procList) == 0 and nextFileToProc == len(filesToProc):
        break;

print "\nHasta la vista, baby."

