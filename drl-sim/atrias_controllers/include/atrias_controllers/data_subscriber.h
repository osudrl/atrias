#ifndef DATA_SUBSCRIBER_H
#define DATA_SUBSCRIBER_H

#include <atrias_controllers/AtriasData.h>
#include <atrias_controllers/data_subscriber_srv.h>

#include "ros/ros.h"

#include <time.h>

FILE *log_file_fp;   // Pointer to logfile.
bool isLogging;   // Should I be logging?

ros::Subscriber data_subscriber;
ros::Publisher data_visualization_publisher;
ros::ServiceServer data_subscriber_srv;


#endif // DATA_SUBSCRIBER_H
