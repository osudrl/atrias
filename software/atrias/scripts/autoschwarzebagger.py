#!/usr/bin/env python2

import sys
import os
import glob
import subprocess

if (len(sys.argv) < 3):
    print("Usage: " + sys.argv[0] + " [directory containing bagfiles to convert] [output directory] [number of threads = 1]")
    exit()

# Get list of files to convert.
os.chdir(sys.argv[1])
filesToProc = glob.glob("*.bag")

print "Processing:"
for f in filesToProc:
    print "    " + f

# Set maximum number of threads to use.
maxNumThreads = 1
if len(sys.argv) > 3:
    maxNumThreads = sys.argv[3]

# Initialize list of processes to spawn.
procList = []
nextFileToProc = 0

while nextFileToProc < len(filesToProc):
    # Spawn off new conversion processes.
    while len(procList) < int(maxNumThreads) and nextFileToProc < len(filesToProc):
        procList.append(subprocess.Popen(["rosrun", "atrias", "fix_bag.py", filesToProc[nextFileToProc], sys.argv[2]+"/"+filesToProc[nextFileToProc]]))
        print "[" + str(nextFileToProc+1) + "/" + str(len(filesToProc)) + "] " + filesToProc[nextFileToProc] + "  using " + str(len(procList)) + "/" + maxNumThreads + " threads"
        nextFileToProc += 1

    # Remove complete processes from list.
    procList = filter(lambda y: y.poll() == None, procList)

