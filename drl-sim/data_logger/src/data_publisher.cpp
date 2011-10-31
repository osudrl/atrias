#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main (int argc, char **argv) {
    ros::init(argc, argv, "data_publisher");
    ros::NodeHandle nh;
    ros::Publisher atrias_datalog_pub = nh.advertise<std_msgs::String>("data_downlink", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        atrias_datalog_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}

