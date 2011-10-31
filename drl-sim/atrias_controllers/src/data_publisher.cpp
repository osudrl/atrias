#include <atrias_controllers/uspace_kern_shm.h>
#include <atrias_controllers/atrias_srv.h>

#include <rtai_shm.h>
#include <rtai_nam2num.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#define QUOTEME_(x) #x
#define QUOTEME(x) QUOTEME_(x)

static Shm * shm;   // shm interface to kernel.

int main (int argc, char **argv) {
    ros::init(argc, argv, "data_publisher");
    ros::NodeHandle nh;
    ros::Publisher atrias_datalog_pub = nh.advertise<std_msgs::String>("data_downlink", 1000);
    ros::Rate loop_rate(10);

    // Connect to kernel's shm.
    rt_allow_nonroot_hrt();   // Allow rt_shm_alloc to be used without root permissions.
    if (!(shm = (Shm*) rt_shm_alloc(nam2num(SHM_NAME), 0, USE_VMALLOC)))
        ROS_ERROR("rtai_malloc() data to user space failed (maybe /dev/rtai_shm is missing)!");
        
    ROS_INFO("Connected to SHM.");

    int i = 0;
    float dataArray[22];

    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;

        ControllerInput* c_in;
        ControllerOutput* c_out;
        if (i != shm->io_index) {
            c_in = &shm->controller_input[i];

            dataArray[0]  = c_in->body_angle;
            dataArray[1]  = c_in->motor_angleA;
            dataArray[2]  = c_in->motor_angleB;
            dataArray[3]  = c_in->leg_angleA;
            dataArray[4]  = c_in->leg_angleB;
            dataArray[5]  = c_in->body_ang_vel;
            dataArray[6]  = c_in->motor_velocityA;
            dataArray[7]  = c_in->motor_velocityB;
            dataArray[8]  = c_in->leg_velocityA;
            dataArray[9]  = c_in->leg_velocityB;
            dataArray[10] = c_in->xPosition;
            dataArray[11] = c_in->yPosition;
            dataArray[12] = c_in->zPosition;
            dataArray[13] = c_in->xVelocity;
            dataArray[14] = c_in->yVelocity;
            dataArray[15] = c_in->zVelocity;
            dataArray[16] = c_out->motor_torqueA;
            dataArray[17] = c_out->motor_torqueB;
            dataArray[18] = c_in->motor_currentA;
            dataArray[19] = c_in->motor_currentB;
            dataArray[20] = c_in->toe_switch;
            dataArray[21] = c_in->command;

            i = (i++) % SHM_TO_USPACE_ENTRIES;
        }

        ss << dataArray[0];
        msg.data = ss.str();

        //ROS_INFO("%s", msg.data.c_str());

        atrias_datalog_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    rt_shm_free(nam2num(SHM_NAME));   // Disconnect from shm.
    return 0;
}

