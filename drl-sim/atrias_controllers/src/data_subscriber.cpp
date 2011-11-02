//! @file data_subscriber.cpp
//! @author Soo-Hyun Yoo
//! @brief Subscribe to data_publisher and log data to file.

#include <atrias_controllers/AtriasData.h>
#include <atrias_msgs/GUIInfo.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

char buffer[256];   // Buffer for general use (e.g., store announced logfile name. TODO: data_subscriber node should decide what filename to use, not GUI.
FILE *log_file_fp;   // Pointer to logfile.

// !@brief Convert float to string for storage. TODO: storage should be in floats that can be parsed by a script to extract user-specified parameters.
std::string format_float(float fl) {
    char charBuf[64];
    sprintf(charBuf, "%.6f", fl);
    std::string buf = charBuf;
    char j;
    bool positive;
    std::string result = "       ";

    for (int i = 0; (j = charBuf[i]) > 0; i++) {   // loop until a null character is encountered
        if (i == 0) {
            if (j == '-') {   // is the number negative
                positive = false;
            }
            else {  
                positive = true;
            }
        }
        else if (j == '.') {
            if (i < 6) {   // if there's room for one or more decimal places, include them
                for (int k = 0; k < 7; k++) {
                    result[k] = charBuf[k];
                }
                return result;
            }
            else {   // if there's no room, leave them out
                for (int k = 0; k < i && k < 7; k++) {   
                    result[k] = charBuf[k];
                }
                return result;
            }
        }
        else if (i > 6) {
            break;
        }
    }
    if (positive) {
        return "NTOOBIG";   // the number is too big too be formatted
    }
    else {
        return "NTOOLOW";   // the number is too small to be formatted
    }
}

//! @brief Log data to logfile.
void datalogCallback(const atrias_controllers::AtriasData &aData) {
    //ROS_INFO("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,%d, %d, %f, %f", aData.time, aData.body_angle, aData.motor_angleA, aData.motor_angleB, aData.leg_angleA, aData.leg_angleB, aData.body_ang_vel, aData.motor_velocityA, aData.motor_velocityB, aData.leg_velocityA, aData.leg_velocityB, aData.xPosition, aData.yPosition, aData.zPosition, aData.xVelocity, aData.yVelocity, aData.zVelocity, aData.horizontal_velocity, aData.motor_currentA, aData.motor_currentB, aData.toe_switch, aData.command, aData.motor_torqueA, aData.motor_torqueB);
    if (log_file_fp == NULL) ROS_ERROR("Logfile is not open.");
    else if (log_file_fp != NULL) {
        fprintf(log_file_fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%d,%d,%s,%s\n",
            format_float(aData.time).c_str(),
            format_float(aData.body_angle).c_str(),
            format_float(aData.motor_angleA).c_str(),
            format_float(aData.motor_angleB).c_str(),
            format_float(aData.leg_angleA).c_str(),
            format_float(aData.leg_angleB).c_str(),
            format_float(aData.body_ang_vel).c_str(),
            format_float(aData.motor_velocityA).c_str(),
            format_float(aData.motor_velocityB).c_str(),
            format_float(aData.leg_velocityA).c_str(),
            format_float(aData.leg_velocityB).c_str(),
            format_float(aData.xPosition).c_str(),
            format_float(aData.yPosition).c_str(),
            format_float(aData.zPosition).c_str(),
            format_float(aData.xVelocity).c_str(),
            format_float(aData.yVelocity).c_str(),
            format_float(aData.zVelocity).c_str(),
            format_float(aData.horizontal_velocity).c_str(),
            format_float(aData.motor_currentA).c_str(),
            format_float(aData.motor_currentB).c_str(),
            format_float(aData.toe_switch).c_str(),
            format_float(aData.command).c_str(),
            format_float(aData.motor_torqueA).c_str(),
            format_float(aData.motor_torqueB).c_str());
    }
}

//! @brief Get logfile name published by GUI and open file to write.
void guiCallback(const atrias_msgs::GUIInfo &guiInfo) {
    sprintf(buffer, guiInfo.logfileName.c_str());
    ROS_INFO("Subscriber got logfile name: %s", buffer);

    log_file_fp = fopen(buffer, "w");
}

//! @brief Main loop.
int main (int argc, char **argv) {
    ros::init(argc, argv, "datalog_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber datalogSubscriber = nh.subscribe("datalog_downlink", 1000, datalogCallback);
    ros::Subscriber guiSubscriber = nh.subscribe("gui_info", 1000, guiCallback);

    while (ros::ok()) {
        ros::spin();
    }
    return 0;
}

