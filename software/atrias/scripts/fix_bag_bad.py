#!/usr/bin/env python2

import rosbag
import genpy.rostime

with rosbag.Bag('output.bag', 'w') as outbag:
    firstMsg = True
    startTime = None
    lastMsg = None
    for topic, msg, t in rosbag.Bag('input.bag').read_messages():
        if topic == "/log_robot_state":
            lastMsg = msg.header.stamp
            if firstMsg:
                firstMsg = False
                startTime = msg.header.stamp
                print("first")
            if msg.header.stamp < startTime:
                startTime = msg.header.stamp
                print("lt")
    print(type(startTime))
    print(type(lastMsg))
    print(type(genpy.rostime.Time(0, (lastMsg - startTime).to_nsec())))
    for topic, msg, t in rosbag.Bag('input.bag').read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        #if topic == "/tf" and msg.transforms:
        #    outbag.write(topic, msg, msg.transforms[0].header.stamp)
        if topic == "/log_robot_state":
            outbag.write(topic, msg, genpy.rostime.Time(0, (msg.header.stamp - startTime).to_nsec()))
