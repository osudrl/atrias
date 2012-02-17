
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
    std::string result = "             ";

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

    return "0.0";
}

//! @brief Log data to logfile.
void datalogCallback(const ros::TimerEvent&) {
    while (i != shm->io_index) {
        c_in = &shm->controller_input[i];
        c_out = &shm->controller_output[i];
        // Publish at about 50 Hz based on data timestamp.
        //if (i % 20 == 0) {
        //    data_visualization_publisher.publish(aData);
        //}
    
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

                    format_float(c_in->time_of_last_stance).c_str(),

                    format_float(c_out->motor_torqueA).c_str(),
                    format_float(c_out->motor_torqueB).c_str(),

                    format_float(c_out->motor_torque_hip).c_str(),

		    format_float(c_in->phase).c_str(),
		    format_float(c_in->desired_motor_position_A).c_str(),
		    format_float(c_in->desired_motor_position_B).c_str(),
		    format_float(c_in->desired_def_A).c_str(),
		    format_float(c_in->desired_def_B).c_str());

            }
            else {
                ROS_ERROR("data_subscriber cannot open log file.");
            }
        }
        i = (i++) % SHM_TO_USPACE_ENTRIES;
        //ROS_INFO("%d %d", i, shm->io_index);
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
            sprintf(buffer, "%s/atrias_%02d%02d%02d_%02d%02d%02d.log", "/mnt/hurst/Experimental_Data/ForceControlCalibration", tInfo->tm_year%100, tInfo->tm_mon+1, tInfo->tm_mday, tInfo->tm_hour, tInfo->tm_min, tInfo->tm_sec);
        }
        else {   // If filename is specified, use that as logfilename.
            ROS_INFO("data_subscriber: Setting logfile name as requested by GUI.");
            sprintf(buffer, "%s", req.logfilename.c_str());
        }

        ROS_INFO("data_subscriber: Opening logfile at %s", buffer);
        log_file_fp = fopen(buffer, "w");   // Open logfile.
        fprintf(log_file_fp, "Time (ms), Body Angle, Body Angle Velocity, Motor A Angle, Motor A Angle (inc), Motor B Angle, Motor B Angle (inc), Leg A Angle, Leg B Angle, Motor A Velocity, Motor B Velocity, Leg A Velocity, Leg B Velocity, Hip Angle, Hip Angular Velocity, X Position, Y Position, Z Position, X Velocity, Y Velocity, Z Velocity, Motor A Current, Motor B Current, Toe Switch, Command, Thermistor A0, Thermistor A1, Thermistor A2, Thermistor B0, Thermistor B1, Thermistor B2, Motor A Voltage, Motor B Voltage, Logic A Voltage, Logic B Voltage, Medulla A Status, Medulla B Status, Time of Last Stance, Motor A Torque, Motor B Torque, Motor Hip Torque, Controller Phase (Stance = 0), Desired Motor Angle A, Desired Motor Angle B, Desired Deflection A, Desired Deflection B\n");   // TODO: Need units for these labels.
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
    // Logging is disabled by default.
    isLogging = false;

    ros::init(argc, argv, "data_subscriber");
    ros::NodeHandle nh;

    // Subscribe to data stream from data_publisher on nettop.
    //data_subscriber = nh.subscribe("datalog_downlink", 0, datalogCallback);

    timer = nh.createTimer(ros::Duration(0.002), datalogCallback);

    rt_allow_nonroot_hrt();
    if (!(shm = (Shm*) rt_shm_alloc(nam2num(SHM_NAME), 0, USE_VMALLOC)))
        ROS_ERROR("rtai_malloc() data to user space failed (maybe /dev/rtai_shm is missing)!");

    ROS_INFO("Datalog publisher connected to shm.");

    // Set i to current value of io_index.
    i = shm->io_index;

    // Provide a much lower-frequency data stream for visualization purposes by
    // re-publishing every few AtriasData fetched by datalogCallback. We
    // wouldn't need to do this if there was a nice way to specify a sampling
    // rate for rxplot, but this workaround suffices for now.
    //
    // TODO: Eventually, GUI should read stuff from this stream instead of
    // asking for data from wireless_kern_interface.

    //data_visualization_publisher = nh.advertise<atrias_controllers::AtriasData>("data_visualization_stream", 0);

    // Service for GUI.
    data_subscriber_srv = nh.advertiseService("data_subscriber_srv", serviceCallback);

    ros::spin();

    // Disconnect from shm.
    rt_shm_free(nam2num(SHM_NAME));

    return 0;
}

