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
torques     = [[float(pair[0])]    for pair in datapairs]
deflections = [[float(pair[1])]    for pair in datapairs]

outfile.write('''
#ifndef PROCESSEDDATA_H
#define PROCESSEDDATA_H

''')

outfile.write('#DEFINE NUM_TORQUE_SAMPLES 2\n')
# The indexes pull the exact value we need from the regression function's output
outfile.write('#DEFINE TORQUE_SAMPLES     {0.0, ' + str(lstsq(deflections, torques)[0][0][0] * max(deflections)[0]) + '}\n')
outfile.write('#DEFINE MAX_DEFLECTION     ' + str(max(deflections)[0]) + '\n')

outfile.write('#DEFINE NUM_DEFL_SAMPLES   2\n')
outfile.write('#DEFINE DEFL_SAMPLES       {0.0, ' + str(lstsq(torques, deflections)[0][0][0] * max(torques)[0])     + '}\n')
outfile.write('#DEFINE MAX_TORQUE         ' + str(max(torques)[0]) + '\n')

outfile.write('''
#endif // PROCESSEDDATA_H
''')

outfile.close()
