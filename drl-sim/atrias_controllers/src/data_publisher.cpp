#include <atrias_controllers/uspace_kern_shm.h>
#include <atrias_controllers/AtriasData.h>

#include <rtai_shm.h>
#include <rtai_nam2num.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#define QUOTEME_(x) #x
#define QUOTEME(x) QUOTEME_(x)

static Shm * shm;   // shm interface to kernel.

int main (int argc, char **argv) {
    ros::init(argc, argv, "datalog_publisher");
    ros::NodeHandle nh;
    ros::Publisher data_publisher = nh.advertise<atrias_controllers::AtriasData>("datalog_downlink", 1000);   // Advertise topic at "data_downlink" and queue up to 1000 messages before dropping the oldest ones.
    ros::Rate loop_rate(1000);   // Loop at 10 Hz.

    // Connect to kernel's shm.
    rt_allow_nonroot_hrt();   // Allow rt_shm_alloc to be used without root permissions.
    if (!(shm = (Shm*) rt_shm_alloc(nam2num(SHM_NAME), 0, USE_VMALLOC)))
        ROS_ERROR("rtai_malloc() data to user space failed (maybe /dev/rtai_shm is missing)!");
        
    ROS_INFO("Datalog publisher connected to shm.");

    ControllerInput* c_in;
    ControllerOutput* c_out;
    int i = 0;
    int dIndex = 0;
    int msgNum = 0;
    atrias_controllers::AtriasData aData;

    while (ros::ok()) {
        if (i != shm->io_index) {   // Check if shm has been updated.
            if (dIndex < 100) {   // Check if dataClump is full.
                c_in = &shm->controller_input[i];
                c_out = &shm->controller_output[i];

                // When adding new fields, make sure to update AtriasData.msg.
                aData.time = msgNum;
                aData.body_angle  = c_in->body_angle;
                aData.motor_angleA = c_in->motor_angleA;
                aData.motor_angleB = c_in->motor_angleB;
                aData.leg_angleA = c_in->leg_angleA;
                aData.leg_angleB = c_in->leg_angleB;

                aData.body_ang_vel = c_in->body_ang_vel;
                aData.motor_velocityA = c_in->motor_velocityA;
                aData.motor_velocityB = c_in->motor_velocityB;
                aData.leg_velocityA = c_in->leg_velocityA;
                aData.leg_velocityB = c_in->leg_velocityB;

                aData.xPosition = c_in->xPosition;
                aData.yPosition = c_in->yPosition;
                aData.zPosition = c_in->zPosition;
                aData.xVelocity = c_in->xVelocity;
                aData.yVelocity = c_in->yVelocity;
                aData.zVelocity = c_in->zVelocity;

                aData.motor_torqueA = c_out->motor_torqueA;
                aData.motor_torqueB = c_out->motor_torqueB;

                aData.horizontal_velocity = c_in->horizontal_velocity;
                aData.motor_currentA = c_in->motor_currentA;
                aData.motor_currentB = c_in->motor_currentB;
                aData.toe_switch = c_in->toe_switch;
                aData.command = c_in->command;

                i = (i++) % SHM_TO_USPACE_ENTRIES;
                dIndex++;
                msgNum++;
            }
            else {
                dIndex = 0;
                //for (i=0; i<100; i++) {
                //    dataClump[i] = 0;
                //}
            }
        }

        data_publisher.publish(aData);
        ros::spinOnce();
        loop_rate.sleep();
    }

    rt_shm_free(nam2num(SHM_NAME));   // Disconnect from shm.
    return 0;
}

