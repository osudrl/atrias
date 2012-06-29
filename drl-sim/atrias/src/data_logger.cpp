//! @file data_logger.cpp
//! @author Soo-Hyun Yoo
//! @brief Subscribe to data_publisher and log data to file.

#include <atrias/data_logger.h>

// !@brief Convert float to string for storage.
//
// TODO: To save disk space, storage should be in floats that can be parsed by
// a script to extract user-specified parameters.
std::string format_float(float fl) {
    char charBuf[64];
    sprintf(charBuf, "%.9f", fl);
    std::string buf = charBuf;
    char j;
    bool positive;
    std::string result = "          ";

    for (int i = 0; (j = charBuf[i]) > 0; i++) {   // loop until a null character is encountered
        if (i == 0) {
            // Check if the number is negative.
            positive = (j == '-') ? false : true;
        }
        else if (j == '.') {
            // If there is room for one or more decimal places, include them.
            // Otherwise, leave them out.
            if (i < 9) {
                for (int k = 0; k < 10; k++) {
                    result[k] = charBuf[k];
                }
                return result;
            }
        }
        else if (i > 9) {
            break;
        }
    }

    return (positive) ? "NTOOBIG" : "NTOOLOW";
}

//! @brief Log data to logfile.
void datalogCallback(const atrias_msgs::controller_status &cs) {
    if (isLogging) {   // This is needed for some reason. Just checking that log_file_fp != NULL allows this node to die.
        if (log_file_fp != NULL) {
            fprintf(log_file_fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
                // command omitted.
                format_float(counter).c_str(),

                // Body
                format_float(cs.state.body_angle).c_str(),
                format_float(cs.state.body_angle_vel).c_str(),
                format_float(cs.state.body_pitch).c_str(),
                format_float(cs.state.body_pitch_vel).c_str(),

                // Leg A
                format_float(cs.state.leg_angleA).c_str(),
                format_float(cs.state.leg_velocityA).c_str(),
                format_float(cs.state.motor_angleA).c_str(),
                format_float(cs.state.motor_angleA_inc).c_str(),
                format_float(cs.state.motor_torqueA).c_str(),
                format_float(cs.state.motor_velocityA).c_str(),
                format_float(cs.state.motorVoltageA).c_str(),
                format_float(cs.state.motor_currentA).c_str(),
                format_float(cs.state.desired_motor_angleA).c_str(),
                format_float(cs.state.desired_spring_defA).c_str(),
                format_float(cs.state.logicVoltageA).c_str(),

                // Leg B
                format_float(cs.state.leg_angleB).c_str(),
                format_float(cs.state.leg_velocityB).c_str(),
                format_float(cs.state.motor_angleB).c_str(),
                format_float(cs.state.motor_angleB_inc).c_str(),
                format_float(cs.state.motor_torqueB).c_str(),
                format_float(cs.state.motor_velocityB).c_str(),
                format_float(cs.state.motorVoltageB).c_str(),
                format_float(cs.state.motor_currentB).c_str(),
                format_float(cs.state.desired_motor_angleB).c_str(),
                format_float(cs.state.desired_spring_defB).c_str(),
                format_float(cs.state.logicVoltageB).c_str(),

                // Hip
                format_float(cs.state.motor_angle_hip).c_str(),
                format_float(cs.state.motor_torque_hip).c_str(),
                format_float(cs.state.motor_velocity_hip).c_str(),

                // Positional
                format_float(cs.state.xPosition).c_str(),
                format_float(cs.state.yPosition).c_str(),
                format_float(cs.state.zPosition).c_str(),
                format_float(cs.state.xVelocity).c_str(),
                format_float(cs.state.yVelocity).c_str(),
                format_float(cs.state.zVelocity).c_str(),
                format_float(cs.state.horizontal_velocity).c_str(),

                // Thermistors
                format_float(cs.state.thermistorA[0]).c_str(),
                format_float(cs.state.thermistorA[1]).c_str(),
                format_float(cs.state.thermistorA[2]).c_str(),
                format_float(cs.state.thermistorB[0]).c_str(),
                format_float(cs.state.thermistorB[1]).c_str(),
                format_float(cs.state.thermistorB[2]).c_str(),

                // medullaStatusA omitted.
                // medullaStatusB omitted.
                // medullaStatusHip omitted.
                // medullaStatusBoom omitted.
                // limitSwitchA omitted.
                // limitSwitchB omitted.
                // limitSwitchHip omitted.

                format_float(cs.state.toe_switch).c_str(),

                // DEBUG DATA

                // Test controller
                format_float(cs.state.phase).c_str(),
                format_float(cs.state.in_flight).c_str(),

                format_float(cs.state.des_leg_angle).c_str(),
                format_float(cs.state.des_leg_length).c_str(),
                format_float(cs.state.des_hip_angle).c_str(),

                format_float(cs.state.flightHipP).c_str(),
                format_float(cs.state.flightHipD).c_str(),
                format_float(cs.state.flightKneeP).c_str(),
                format_float(cs.state.flightKneeD).c_str(),

                format_float(cs.state.stanceHipP).c_str(),
                format_float(cs.state.stanceHipD).c_str(),
                format_float(cs.state.stanceKneeP).c_str(),
                format_float(cs.state.stanceKneeD).c_str(),

                format_float(cs.state.stance_time).c_str());
        }
        else {
            ROS_ERROR("data_logger cannot open log file.");
        }

        counter++;
    }
}

//! @brief Get logfile name published by GUI and open file to write.
bool serviceCallback(atrias_msgs::data_log_srv::Request& req, atrias_msgs::data_log_srv::Response& res) {
    if (req.isLogging) {
        ROS_INFO("[data_logger] Logging.");
        char buffer[256];   // Create buffer to store filename.
        time_t curSeconds =  time(NULL);
        struct tm *tInfo = localtime(&curSeconds);

        if (req.logfilename == "") {   // If filename is unspecified, set one based on date and time.
            ROS_INFO("[data_logger] Logfile name unspecified. Deciding logfile name based on date and time.");
// Hubicki, added option to change directory - 02-10-2012
//            sprintf(buffer, "%s/atrias_preempt_rt_%02d%02d%02d_%02d%02d%02d.log", "/home/drl/atrias/drl-sim/atrias/log_files", tInfo->tm_year%100, tInfo->tm_mon+1, tInfo->tm_mday, tInfo->tm_hour, tInfo->tm_min, tInfo->tm_sec);

	    sprintf(buffer, "%s/atrias_preempt_rt_%02d%02d%02d_%02d%02d%02d.log", "/mnt/hurst/Experimental_Data/ForceControlCalibration", tInfo->tm_year%100, tInfo->tm_mon+1, tInfo->tm_mday, tInfo->tm_hour, tInfo->tm_min, tInfo->tm_sec);
// End Hubicki change


        }
        else {   // If filename is specified, use that as logfilename.
            ROS_INFO("[data_logger] Setting logfile name as requested by GUI.");
            sprintf(buffer, "%s", req.logfilename.c_str());
        }

        try {
            log_file_fp = fopen(buffer, "w");   // Open logfile.
            if (log_file_fp == NULL) {
                throw "[data_logger] /mnt/hurst is not mounted!";
            }
        }
        catch (char const* str) {
            ROS_INFO(str);
	        sprintf(buffer, "%s/atrias_preempt_rt_%02d%02d%02d_%02d%02d%02d.log", "/home/drl/newgui/drl-sim/atrias/log_files", tInfo->tm_year%100, tInfo->tm_mon+1, tInfo->tm_mday, tInfo->tm_hour, tInfo->tm_min, tInfo->tm_sec);

            try {
                log_file_fp = fopen(buffer, "w");   // Try opening again.
                if (log_file_fp == NULL) {
                    throw "[data_logger] /home/drl/newgui/drl-sim/atrias/log_files is inaccessible!";
                }
            }
            catch (char const* str) {
                ROS_INFO(str);
                ROS_WARN("[data_logger] Not logging!");
            }
        }
        ROS_INFO("[data_logger] Opening logfile at %s", buffer);

        fprintf(log_file_fp, "Time (ms), Body Angle, Body Angle Velocity, Body Pitch, Body Pitch Velocity, Leg A Angle, Leg A Velocity, Leg A Motor Angle, Leg A Motor Angle (inc), Leg A Motor Torque, Leg A Motor Velocity, Leg A Motor Voltage, Leg A Motor Current, Leg A Desired Motor Angle, Leg A Desired Spring Deflection, Leg A Logic Voltage, Leg B Angle, Leg B Velocity, Leg B Motor Angle, Leg B Motor Angle (inc), Leg B Motor Torque, Leg B Motor Velocity, Leg B Motor Voltage, Leg B Motor Current, Leg B Desired Motor Angle, Leg B Desired Spring Deflection, Leg B Logic Voltage, Hip Motor Angle, Hip Motor Torque, Hip Motor Velocity, X Position, Y Position, Z Position, X Velocity, Y Velocity, Z Velocity, Horizontal Velocity, Thermistor A0, Thermistor A1, Thermistor A2, Thermistor B0, Thermistor B1, Thermistor B2, Toe Switch, Phase, In Flight, Desired Leg Angle, Desired Leg Length, Desired Hip Angle, Flight Hip P, Flight Hip D, Flight Knee P, Flight Knee D, Stance Hip P, Stance Hip D, Stance Knee P, Stance Knee D, Stance Time\n");   // TODO: Need units for these labels.
        res.logfilename = buffer;   // Respond with new logfilename.
        isLogging = true;   // data_logger should start logging.
        counter = 0;
    }
    else if (!req.isLogging) {
        ROS_INFO("[data_logger] Not logging.");
        if (log_file_fp != NULL) {   // Check if logfile is open because serviceCallback could be run if logfilename is set but isLogging is off.
            fclose(log_file_fp);
            ROS_INFO("[data_logger] Closing logfile.");
        }
        res.logfilename = "";   // Respond with blank logfilename.
        isLogging = false;
    }
    return true;
}

//! @brief Main loop.
int main (int argc, char **argv) {
    // Logging is disabled by default.
    isLogging = false;

    counter = 0;

    ros::init(argc, argv, "data_logger");
    ros::NodeHandle nh;

    // Subscribe to data stream from data_publisher on nettop.
    data_logger = nh.subscribe("controller_status_1000_hz", 0, datalogCallback);

    // Service for GUI.
    data_log_srv = nh.advertiseService("data_log_srv", serviceCallback);

    // Run ROS loop.
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}

