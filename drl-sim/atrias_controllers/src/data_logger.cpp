
#include <atrias_controllers/data_logger.h>

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
void datalogCallback(const ros::TimerEvent&) {
    while (i != shm->io_index) {
        c_in = &shm->controller_input[i];
        c_out = &shm->controller_output[i];
        // Publish at about 50 Hz based on data timestamp.
        if (i % 20 == 0) {
            // When adding new fields, make sure to update AtriasData.msg.
            aData.time = i;
    
            aData.body_angle     = c_in->body_angle;
            aData.body_angle_vel = c_in->body_angle_vel;
            aData.motor_angleA     = c_in->motor_angleA;
            aData.motor_angleA_inc = c_in->motor_angleA_inc;
            aData.motor_angleB     = c_in->motor_angleB;
            aData.motor_angleB_inc = c_in->motor_angleB_inc;
            aData.leg_angleA = c_in->leg_angleA;
            aData.leg_angleB = c_in->leg_angleB;
    
            aData.motor_velocityA = c_in->motor_velocityA;
            aData.motor_velocityB = c_in->motor_velocityB;
            aData.leg_velocityA   = c_in->leg_velocityA;
            aData.leg_velocityB   = c_in->leg_velocityB;
    
            aData.hip_angle     = c_in->hip_angle;
            aData.hip_angle_vel = c_in->hip_angle_vel;
    
            aData.xPosition = c_in->xPosition;
            aData.yPosition = c_in->yPosition;
            aData.zPosition = c_in->zPosition;
    
            aData.xVelocity = c_in->xVelocity;
            aData.yVelocity = c_in->yVelocity;
            aData.zVelocity = c_in->zVelocity;
    
            aData.motor_currentA = c_in->motor_currentA;
            aData.motor_currentB = c_in->motor_currentB;
    
            aData.toe_switch = c_in->toe_switch;
    
            aData.command = c_in->command;
    
            aData.thermistorA[0] = c_in->thermistorA[0];
            aData.thermistorA[1] = c_in->thermistorA[1];
            aData.thermistorA[2] = c_in->thermistorA[2];
            aData.thermistorB[0] = c_in->thermistorB[0];
            aData.thermistorB[1] = c_in->thermistorB[1];
            aData.thermistorB[2] = c_in->thermistorB[2];
            aData.motorVoltageA = c_in->motorVoltageA;
            aData.motorVoltageB = c_in->motorVoltageB;
            aData.logicVoltageA = c_in->logicVoltageA;
            aData.logicVoltageB = c_in->logicVoltageB;
            aData.medullaStatusA = c_in->medullaStatusA;
            aData.medullaStatusB = c_in->medullaStatusB;
    
            aData.time_of_last_stance = c_in->stance_time;
    
            aData.motor_torqueA = c_out->motor_torqueA;
            aData.motor_torqueB = c_out->motor_torqueB;
            aData.motor_torque_hip = c_out->motor_torque_hip;

            aData.phase = c_in->phase;
            aData.desired_motor_angleA = c_in->desired_motor_angleA;
            aData.desired_motor_angleB = c_in->desired_motor_angleB;
            aData.desired_spring_defA  = c_in->desired_spring_defA;
            aData.desired_spring_defB  = c_in->desired_spring_defB;


            // Publish the data. This takes up the majority of the loop time.
            data_publisher.publish(aData);
        }
    
        if (isLogging) {   // This is needed for some reason. Just checking that log_file_fp != NULL allows this node to die.
            if (log_file_fp != NULL) {
                fprintf(log_file_fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
                    format_float(i).c_str(),
    
                    format_float(c_in->body_angle).c_str(),
                    format_float(c_in->body_angle_vel).c_str(),
                    format_float(c_in->motor_angleA).c_str(),
                    format_float(c_in->motor_angleA_inc).c_str(),
                    format_float(c_in->motor_angleB).c_str(),
                    format_float(c_in->motor_angleB_inc).c_str(),
                    format_float(c_in->leg_angleA).c_str(),
                    format_float(c_in->leg_angleB).c_str(),
    
                    format_float(c_in->motor_velocityA).c_str(),
                    format_float(c_in->motor_velocityB).c_str(),
                    format_float(c_in->leg_velocityA).c_str(),
                    format_float(c_in->leg_velocityB).c_str(),
    
                    format_float(c_in->hip_angle).c_str(),
                    format_float(c_in->hip_angle_vel).c_str(),
    
                    format_float(c_in->xPosition).c_str(),
                    format_float(c_in->yPosition).c_str(),
                    format_float(c_in->zPosition).c_str(),
    
                    format_float(c_in->xVelocity).c_str(),
                    format_float(c_in->yVelocity).c_str(),
                    format_float(c_in->zVelocity).c_str(),
    
                    format_float(c_in->motor_currentA).c_str(),
                    format_float(c_in->motor_currentB).c_str(),
    
                    format_float(c_in->toe_switch).c_str(),
    
                    format_float(c_in->command).c_str(),
    
                    format_float(c_in->thermistorA[0]).c_str(),
                    format_float(c_in->thermistorA[1]).c_str(),
                    format_float(c_in->thermistorA[2]).c_str(),
                    format_float(c_in->thermistorB[0]).c_str(),
                    format_float(c_in->thermistorB[1]).c_str(),
                    format_float(c_in->thermistorB[2]).c_str(),
                    format_float(c_in->motorVoltageA).c_str(),
                    format_float(c_in->motorVoltageB).c_str(),
                    format_float(c_in->logicVoltageA).c_str(),
                    format_float(c_in->logicVoltageB).c_str(),
                    format_float(c_in->medullaStatusA).c_str(),
                    format_float(c_in->medullaStatusB).c_str(),
    
                    format_float(c_in->stance_time).c_str(),
    
                    format_float(c_out->motor_torqueA).c_str(),
                    format_float(c_out->motor_torqueB).c_str(),
                    format_float(c_out->motor_torque_hip).c_str(),

		    format_float(c_in->phase).c_str(),
                    format_float(c_in->desired_motor_angleA).c_str(),
                    format_float(c_in->desired_motor_angleB).c_str(),
                    format_float(c_in->desired_spring_defA).c_str(),
                    format_float(c_in->desired_spring_defB).c_str());
            }
            else {
                ROS_ERROR("data_subscriber cannot open log file.");
            }
        }
        i = (i+1) % SHM_TO_USPACE_ENTRIES;
        ROS_INFO("%d %d", i, shm->io_index);
    }
    ROS_INFO("shm not updated yet!");
}

//! @brief Get logfile name published by GUI and open file to write.
bool serviceCallback(atrias_controllers::data_subscriber_srv::Request& req, atrias_controllers::data_subscriber_srv::Response& res) {
    if (req.isLogging) {
        ROS_INFO("[data_subscriber] Logging.");
        char buffer[256];   // Create buffer to store filename.
        time_t curSeconds =  time(NULL);
        struct tm *tInfo = localtime(&curSeconds);

        if (req.logfilename == "") {   // If filename is unspecified, set one based on date and time.
            ROS_INFO("[data_subscriber] Logfile name unspecified. Deciding logfile name based on date and time.");
// Hubicki, added option to change directory - 02-10-2012
//            sprintf(buffer, "%s/atrias_%02d%02d%02d_%02d%02d%02d.log", "/home/drl/atrias/drl-sim/atrias/log_files", tInfo->tm_year%100, tInfo->tm_mon+1, tInfo->tm_mday, tInfo->tm_hour, tInfo->tm_min, tInfo->tm_sec);

            sprintf(buffer, "%s/atrias_%02d%02d%02d_%02d%02d%02d.log", "/mnt/hurst/Experimental_Data/ForceControlCalibration", tInfo->tm_year%100, tInfo->tm_mon+1, tInfo->tm_mday, tInfo->tm_hour, tInfo->tm_min, tInfo->tm_sec);
// End Hubicki change


        }
        else {   // If filename is specified, use that as logfilename.
            ROS_INFO("[data_subscriber] Setting logfile name as requested by GUI.");
            sprintf(buffer, "%s", req.logfilename.c_str());
        }

        try {
            log_file_fp = fopen(buffer, "w");   // Open logfile.
            if (log_file_fp == NULL) {
                throw "[data_subscriber] /mnt/hurst is not mounted!";
            }
        }
        catch (char const* str) {
            ROS_INFO(str);
                sprintf(buffer, "%s/atrias_%02d%02d%02d_%02d%02d%02d.log", "/home/drl/atrias/drl-sim/atrias/log_files", tInfo->tm_year%100, tInfo->tm_mon+1, tInfo->tm_mday, tInfo->tm_hour, tInfo->tm_min, tInfo->tm_sec);
            log_file_fp = fopen(buffer, "w");   // Try opening again.
        }
        ROS_INFO("[data_subscriber] Opening logfile at %s", buffer);

        fprintf(log_file_fp, "Time (ms), Body Angle, Body Angle Velocity, Motor A Angle, Motor A Angle (inc), Motor B Angle, Motor B Angle (inc), Leg A Angle, Leg B Angle, Motor A Velocity, Motor B Velocity, Leg A Velocity, Leg B Velocity, Hip Angle, Hip Angular Velocity, X Position, Y Position, Z Position, X Velocity, Y Velocity, Z Velocity, Motor A Current, Motor B Current, Toe Switch, Command, Thermistor A0, Thermistor A1, Thermistor A2, Thermistor B0, Thermistor B1, Thermistor B2, Motor A Voltage, Motor B Voltage, Logic A Voltage, Logic B Voltage, Medulla A Status, Medulla B Status, Time of Last Stance, Motor A Torque, Motor B Torque, Motor Hip Torque, Phase, Desired Motor Angle A, Desired Motor Angle B, Desired Spring Deflection A, Desired Spring Deflection B\n");   // TODO: Need units for these labels.
        res.logfilename = buffer;   // Respond with new logfilename.
        isLogging = true;   // data_subscriber should start logging.
    }
    else if (!req.isLogging) {
        ROS_INFO("[data_subscriber] Not logging.");
        if (log_file_fp != NULL) {   // Check if logfile is open because serviceCallback could be run if logfilename is set but isLogging is off.
            fclose(log_file_fp);
            ROS_INFO("[data_subscriber] Closing logfile.");
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

    ros::init(argc, argv, "data_logger");

    // Initialize the nodehandle. This must be called after ros::init.
    ros::NodeHandle nh;

    // Advertise the Publisher.
    //
    // atrias_controllers::AtriasData - Format of ROS msg to publish.
    // "atrias_data_50_hz" - Name of topic to publish. ROS Subscribers can
    //                       listen to what this Publisher publishes by
    //                       subscribing to this topic.
    data_publisher = nh.advertise<atrias_msgs::atrias_data>("atrias_data_50_hz", 0);

    // Initialize the Timer. (NOTE: This is NOT a replacement for realtime.
    // There are no guarantees about how accurate this is.)
    //
    // ros::Duration - ROS primitive type representing a period of Time.
    //                 i.e., Time is a specific moment (e.g., 5:00 pm) whereas
    //                 Duration is a period of Time (e.g., 5 hours).
    // 0.002 - 2 milliseconds. The timer duration shouldn't be too critical, as
    //         datalogCallback uses a while (not if) loop to check for io_index
    //         updates.
    // datalogCallback - function to call every duration..
    timer = nh.createTimer(ros::Duration(0.002), datalogCallback);

    // Allow rt_shm_alloc to be used without root permissions.
    rt_allow_nonroot_hrt();

    // Attempt to allocate shared memory space.
    //
    // rt_shm_alloc() - Allocation function declared in rtai_shm.h.
    // SHM_NAME - Defined in uspace_kern_shm.h
    // 0 - Allocation size can be specified, but USE_VMALLOC makes this
    //     unnecessary.
    // USE_VMALLOC - A define passed to rt_shm_alloc for.. I don't know.
    //               TODO: Need to understand this better.
    if (!(shm = (Shm*) rt_shm_alloc(nam2num(SHM_NAME), 0, USE_VMALLOC)))
        ROS_ERROR("rtai_malloc() data to user space failed (maybe /dev/rtai_shm is missing)!");

    ROS_INFO("Data logger connected to shm.");

    // Set i to current value of io_index.
    i = shm->io_index;

    // Service for GUI.
    data_subscriber_srv = nh.advertiseService("data_subscriber_srv", serviceCallback);

    ros::spin();

    // Disconnect from shm.
    rt_shm_free(nam2num(SHM_NAME));

    return 0;
}

