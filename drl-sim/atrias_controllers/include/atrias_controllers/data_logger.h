// Devin Koepl

#ifndef FUNCS_H_DATA_LOGGER
#define FUNCS_H_DATA_LOGGER

#include <atrias_controllers/uspace_kern_shm.h>
#include <atrias_controllers/data_subscriber_srv.h>

#include <rtai_shm.h>
#include <rtai_nam2num.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <time.h>

#define QUOTEME_(x) #x
#define QUOTEME(x) QUOTEME_(x)

FILE *log_file_fp;   // Pointer to logfile.
bool isLogging;   // Should I be logging?

ros::Subscriber data_subscriber;
//ros::Publisher data_visualization_publisher;
ros::ServiceServer data_subscriber_srv;

static Shm * shm;   // shm interface to kernel.

ros::Timer timer;

ControllerInput* c_in;
ControllerOutput* c_out;

int i;

#endif // FUNCS_H_DATA_LOGGER
