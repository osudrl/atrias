#ifndef DATA_PUBLISHER_H
#define DATA_PUBLISHER_H

#include <atrias_controllers/uspace_kern_shm.h>
#include <atrias_msgs/atrias_data.h>

#include <rtai_shm.h>
#include <rtai_nam2num.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#define QUOTEME_(x) #x
#define QUOTEME(x) QUOTEME_(x)

static Shm * shm;   // shm interface to kernel.

ros::Publisher data_publisher;

ros::Timer timer;

ControllerInput* c_in;
ControllerOutput* c_out;

int i;

// Instantiate ROS msg AtriasData. Publisher will use this format to publish
// over the ROS network.
atrias_msgs::atrias_data aData;


#endif // DATA_PUBLISHER_H
