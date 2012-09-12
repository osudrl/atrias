#!/usr/bin/env python2

import roslib; roslib.load_manifest("atrias_msgs")
import rospy
from signal import signal, SIGINT
from atrias_msgs.msg import log_request
from os import system, fork, getpid

currentlyLogging = False

def logRequestCallback (req):
    global currentlyLogging
    if req.enableLogging and not currentlyLogging:
        child_pid = fork()
        if child_pid == 0:
            print "Child process: PID# %s" % getpid()
            system("roslaunch atrias rosbag.launch")
        else:
            print "Parent process: PID# %s" % getpid()
            print "Launched rosbag."
            currentlyLogging = True
            print "Currently logging."

    elif not req.enableLogging and currentlyLogging:
        child_pid = fork()
        if child_pid == 0:
            print "Child process: PID# %s" % getpid()
            system("rosnode kill atrias_rosbag")
        else:
            print "Parent process: PID# %s" % getpid()
            print "Tried to kill rosbag."
            currentlyLogging = False
            print "I don't think I'm logging anymore."

def printMsg():
    print "Logger shutting down..."

if __name__ == "__main__":
    # Initialize ROS node.
    rospy.init_node("atrias_logger", anonymous=False)
    sub = rospy.Subscriber("atrias_log_request", log_request, logRequestCallback, queue_size=10)
    rospy.spin()

    rospy.on_shutdown(printMsg)

