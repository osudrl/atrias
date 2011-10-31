#include "ros/ros.h"
#include "std_msgs/String.h"

void dataCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "data_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("data_downlink", 1000, dataCallback);

    ros::spin();

    return 0;
}

