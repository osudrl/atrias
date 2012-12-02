#!/usr/bin/env python2

# This script parses in the spring tester data and outputs a .h
# file containing arrays and sizes for a linear interpolation
# controller. The first parameter should be file with the data
# and the second should be the header file into which
# to dump the data.

import sys
from numpy.linalg import lstsq

outfile = open(sys.argv[2], 'w')

datapairs   = [line.split() for line in open(sys.argv[1])]
torques     = [[pair[0]]    for pair in datapairs]
deflections = [[pair[1]]    for pair in datapairs]

print('''
#ifndef PROCESSEDDATA_H
#define PROCESSEDDATA_H

''')

print('#DEFINE NUM_TORQUE_SAMPLES 2')
# The indexes pull the exact value we need from the regression function's output
print(max(deflections))
print('#DEFINE TORQUE_SAMPLES     { 0.0, ' + str(lstsq(deflections, torques)[0][0][0] * max(deflections)[0]))
print('#DEFINE MAX_DEFLECTION     ' + str(max(deflections)))

outfile.close()
