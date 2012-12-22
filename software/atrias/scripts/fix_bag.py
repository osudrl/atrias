#!/usr/bin/env python2

import rosbag
import genpy.rostime
import sys

if (len(sys.argv) < 3):
	print("Usage: " + sys.argv[0] + " [infile.bag] [outfile.bag]")
	exit()

# This is the difference between our "fake epoch", taken from the first message
# with a (valid) header and the real epoch.
# real epoch + fakeEpochOffset = fake epoch
fakeEpochOffset = 0

# This is the difference between the header times and the "fake epoch"
# header time + header Epoch Diff = fake epoch timestamp
headerEpochDiff = None

# Convenience function to converts nanoseconds since an epoch into a
# ROS Time object.
def toTime(nanoseconds):
	return genpy.rostime.Time(int(nanoseconds // 1e9), nanoseconds % 1e9);

with rosbag.Bag(sys.argv[2], 'w') as outbag:
	for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():
		# Check not only if it has a header, but if that header has been populated
		if msg._has_header and msg.header.stamp.to_nsec() != 0:
			if headerEpochDiff is None:
				# Let's set this up
				headerEpochDiff = t.to_nsec() - msg.header.stamp.to_nsec();
			
			# Let's update fakeEpochOffset
			fakeEpochOffset = min(fakeEpochOffset, t.to_nsec() - msg.header.stamp.to_nsec() - headerEpochDiff);
			
			# and output the new message
			outbag.write(topic, msg, toTime(msg.header.stamp.to_nsec() + headerEpochDiff));
		else:
			outbag.write(topic, msg, toTime(t.to_nsec() - fakeEpochOffset));
