#!/usr/bin/env python2

import sys
import os

print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
print sys.argv
print os.getcwd()
print "################################"

file = open(sys.argv[2], 'w')
file.write("Stuffz\n")
file.close()
