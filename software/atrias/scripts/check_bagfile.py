#!/usr/bin/env python2

import rosbag
import genpy.rostime
import sys

if (len(sys.argv) < 2):
	print("Usage: " + sys.argv[0] + " [file.bag]")
	exit()

# This script checks for jumps in sequence numbers for log_robot_state

first_msg = True;
seq_num = 0;

for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():
	if (topic != "/log_robot_state"):
		continue;
	
	seq_num = seq_num + 1;

	if (first_msg):
		first_msg = False;
		seq_num = msg.header.seq;
	
	if (msg.header.seq != seq_num):
		print("Sequence skipped by: " + str(msg.header.seq - seq_num));
		seq_num = msg.header.seq;
