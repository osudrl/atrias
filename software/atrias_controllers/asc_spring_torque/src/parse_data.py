#!/usr/bin/env python2

# This script parses in the spring tester data and outputs a .h
# file containing arrays and sizes for a linear interpolation
# controller. The first parameter should be file with the data
# and the second should be the header file into which
# to dump the data.

# Configuration
# This is the max torque we may command (just for torque->deflection calcs.). In Newton*meters
maxTorque = 1000

import sys
from numpy.linalg import lstsq

outfile = open(sys.argv[2], 'w')

datapairs   = [line.split() for line in open(sys.argv[1])]
torques     = [[float(pair[0])] for pair in datapairs]
deflections = [[float(pair[1])] for pair in datapairs]

outfile.write('''
#ifndef PROCESSEDDATA_H
#define PROCESSEDDATA_H

''')

# This is our spring constant -- assuming a linear spring
sprConst = lstsq(deflections, torques)[0][0][0];

outfile.write('double deflToConst(double defl) {\n')
outfile.write('	return ' + str(sprConst) + ';\n')
outfile.write('}\n\n')

outfile.write('double deflToTorque(double defl) {\n')
outfile.write('	return ' + str(sprConst) + ' * defl;\n')
outfile.write('}\n\n')

outfile.write('double trqToDefl(double trq) {\n')
outfile.write('	if (trq > ' + str(maxTorque) + ')\n')
outfile.write('		trq = ' + str(maxTorque) + ';\n')
outfile.write('	else if (trq < ' + str(-maxTorque) + ')\n')
outfile.write('		trq = ' + str(-maxTorque) + ';\n')
outfile.write('\n')
outfile.write('	return trq * ' + str(1 / sprConst) + ';\n')
outfile.write('}\n')

outfile.write('''
#endif // PROCESSEDDATA_H
''')

outfile.close()
