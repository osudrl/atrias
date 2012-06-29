#ifndef DATA_SUBSCRIBER_H
#define DATA_SUBSCRIBER_H

#include <atrias_msgs/controller_status.h>
#include <atrias_msgs/data_log_srv.h>

#include "ros/ros.h"

#include <time.h>

unsigned long int counter;

FILE *log_file_fp;   // Pointer to logfile.
bool isLogging;   // Should I be logging?

ros::Subscriber data_logger;
ros::ServiceServer data_log_srv;

#endif // DATA_SUBSCRIBER_H
