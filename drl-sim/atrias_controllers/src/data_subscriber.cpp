#include <atrias_controllers/AtriasData.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

int i=0;

void dataCallback(const atrias_controllers::AtriasData &data) {
    ROS_INFO("%i", i);
    i++;
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "datalog_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("datalog_downlink", 1000, dataCallback);

    ros::spin();

    return 0;
}

