#!/usr/bin/env python2

import roslib; roslib.load_manifest("atrias_msgs")
import rospy
from signal import signal, SIGINT
from atrias_msgs.msg import log_request
from os import system

currentlyLogging = False

def logRequestCallback (req):
    global currentlyLogging
    if req and not currentlyLogging:
        system("roslaunch atrias rosbag.launch")
    elif not req and currentlyLogging:
        system("rosnode kill atrias_rosbag")

if __name__ == "__main__":
    # Initialize ROS node.
    rospy.init_node("atrias_logger", anonymous=False)
    sub = rospy.Subscriber("atrias_log_request", log_request, logRequestCallback, queue_size=10)

