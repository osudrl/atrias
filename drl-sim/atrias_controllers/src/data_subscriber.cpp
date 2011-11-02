#include <atrias_controllers/AtriasData.h>
#include <atrias_msgs/GUIInfo.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

char buffer[256];

void datalogCallback(const atrias_controllers::AtriasData &aData) {
    //ROS_INFO("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,%d, %d, %f, %f", aData.time, aData.body_angle, aData.motor_angleA, aData.motor_angleB, aData.leg_angleA, aData.leg_angleB, aData.body_ang_vel, aData.motor_velocityA, aData.motor_velocityB, aData.leg_velocityA, aData.leg_velocityB, aData.xPosition, aData.yPosition, aData.zPosition, aData.xVelocity, aData.yVelocity, aData.zVelocity, aData.horizontal_velocity, aData.motor_currentA, aData.motor_currentB, aData.toe_switch, aData.command, aData.motor_torqueA, aData.motor_torqueB);
}

void guiCallback(const atrias_msgs::GUIInfo &guiInfo) {

    sprintf(buffer, guiInfo.logfileName.c_str());
    ROS_INFO("Subscriber got logfile name: %s", buffer);

}

int main (int argc, char **argv) {
    ros::init(argc, argv, "datalog_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber datalogSubscriber = nh.subscribe("datalog_downlink", 1000, datalogCallback);

    ros::Subscriber guiSubscriber = nh.subscribe("gui_info", 1000, guiCallback);

    ros::spin();

    return 0;
}

