#!/bin/bash
# Script to profile a system for debugging
# This is currently just an outline that needs to be filled in
# Written by Andrew Peekema


# System information is located at /proc (see "man proc")

# Find how many clock ticks per second there are since
# the information in /proc/stat is measured in this
ticksPerSec=$(getconf CLK_TCK)

# Initial cpu idle time
# TODO
idleTime0=$(cat /proc/stat)

# Loop
while true; do
    # Grab the time (in nanoseconds)
    absTime=$(cat /proc/timer_list | grep "now at [0-9]* nsecs" | sed 's/now at //; s/ nsecs//')

    # Idle cpu time (all cores)
    # TODO
    idleTime=$(cat /proc/stat)

    # rosbag CPU usage

    # Disk I/O Usage

    # Network (loopback) utilization

    # Write to file

    # Wait for 10 ms
    sleep 0.01
done
