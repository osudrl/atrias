//! @file data_subscriber.cpp
//! @author Soo-Hyun Yoo
//! @brief Subscribe to data_publisher and log data to file.

#include <atrias_controllers/AtriasData.h>
#include <atrias_controllers/data_subscriber_srv.h>

#include "ros/ros.h"

#include <time.h>

FILE *log_file_fp;   // Pointer to logfile.
bool isLogging = false;   // Should I be logging?

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
    if (isLogging) {   // This is needed for some reason. Just checking that log_file_fp != NULL allows this node to die.
        if (log_file_fp != NULL) {
            fprintf(log_file_fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
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
        else {
            ROS_ERROR("data_subscriber cannot open log file.");
        }
    }
}

//! @brief Get logfile name published by GUI and open file to write.
bool serviceCallback(atrias_controllers::data_subscriber_srv::Request& req, atrias_controllers::data_subscriber_srv::Response& res) {
    if (req.isLogging) {
        ROS_INFO("data_subscriber: Logging.");
        char buffer[256];   // Create buffer to store filename.

        if (req.logfilename == "") {   // If filename is unspecified, set one based on date and time.
            ROS_INFO("data_subscriber: Logfile name unspecified. Deciding logfile name based on date and time.");
            time_t curSeconds;
            curSeconds = time(NULL);
            struct tm *tInfo;
            tInfo = localtime(&curSeconds);
            sprintf(buffer, "%s/atrias_%02d%02d%02d_%02d%02d%02d.log", "/home/drl/atrias/drl-sim/atrias/log_files", tInfo->tm_year%100, tInfo->tm_mon+1, tInfo->tm_mday, tInfo->tm_hour, tInfo->tm_min, tInfo->tm_sec);
        }
        else {   // If filename is specified, use that as logfilename.
            ROS_INFO("data_subscriber: Setting logfile name as requested by GUI.");
            sprintf(buffer, "%s", req.logfilename.c_str());
        }

        ROS_INFO("data_subscriber: Opening logfile at %s", buffer);
        log_file_fp = fopen(buffer, "w");   // Open logfile.
        fprintf(log_file_fp, "Time (ms), Body Angle, Motor A Angle, Motor B Angle, Leg A Angle, Leg B Angle, Body Angular Velocity, Motor A Velocity, Motor B Velocity, Leg A Velocity, Leg B Velocity, X Position, Y Position, Z Position, X Velocity, Y Velocity, Z Velocity, Horizontal Velocity, Motor A Current, Motor B Current, Toe Switch, Command, Motor A Torque, Motor B Torque");   // TODO: Need units for these labels.
        res.logfilename = buffer;   // Respond with new logfilename.
        isLogging = true;   // data_subscriber should start logging.
    }
    else if (!req.isLogging) {
        ROS_INFO("data_subscriber: Not logging.");
        if (log_file_fp != NULL) {   // Check if logfile is open because serviceCallback could be run if logfilename is set but isLogging is off.
            fclose(log_file_fp);
            ROS_INFO("data_subscriber: Closing logfile.");
        }
        res.logfilename = "";   // Respond with blank logfilename.
        isLogging = false;
    }
    return true;
}

//! @brief Main loop.
int main (int argc, char **argv) {
    ros::init(argc, argv, "data_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber data_subscriber = nh.subscribe("datalog_downlink", 1000, datalogCallback);
    ros::ServiceServer data_subscriber_srv = nh.advertiseService("data_subscriber_srv", serviceCallback);

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}

