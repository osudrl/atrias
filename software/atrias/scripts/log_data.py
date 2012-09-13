#!/usr/bin/env python2

import roslib; roslib.load_manifest("atrias_msgs")
import rospy
from atrias_msgs.msg import log_request
import subprocess


roslaunchProcess = 0
currentlyLogging = False


def logRequestCallback (req):
    global roslaunchProcess, currentlyLogging

    if req.enableLogging and not currentlyLogging:
        roslaunchProcess = subprocess.Popen(["roslaunch", "atrias", "rosbag.launch"])
        currentlyLogging = True

    elif not req.enableLogging and currentlyLogging:
        killProcess = subprocess.Popen(["rosnode", "kill", "atrias_rosbag"])
        roslaunchProcess.wait()   # Wait for processes to die, and reap them.
        killProcess.wait()
        currentlyLogging = False


if __name__ == "__main__":
    # Initialize ROS node.
    rospy.init_node("atrias_logger", anonymous=False)
    sub = rospy.Subscriber("atrias_log_request", log_request, logRequestCallback, queue_size=10)
    rospy.spin()

