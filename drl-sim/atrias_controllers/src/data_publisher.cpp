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
    atrias_controllers::AtriasData dataClump;

    while (ros::ok()) {
        if (i != shm->io_index) {   // Check if shm has been updated.
            if (dIndex < 100) {   // Check if dataClump is full.
                c_in = &shm->controller_input[i];
                c_out = &shm->controller_output[i];

                // When adding new fields, make sure to update AtriasData.msg.
                dataClump.body_angle  = c_in->body_angle;
                //dataArray[1]  = c_in->motor_angleA;
                //dataArray[2]  = c_in->motor_angleB;
                //dataArray[3]  = c_in->leg_angleA;
                //dataArray[4]  = c_in->leg_angleB;
                //dataArray[5]  = c_in->body_ang_vel;
                //dataArray[6]  = c_in->motor_velocityA;
                //dataArray[7]  = c_in->motor_velocityB;
                //dataArray[8]  = c_in->leg_velocityA;
                //dataArray[9]  = c_in->leg_velocityB;
                //dataArray[10] = c_in->xPosition;
                //dataArray[11] = c_in->yPosition;
                //dataArray[12] = c_in->zPosition;
                //dataArray[13] = c_in->xVelocity;
                //dataArray[14] = c_in->yVelocity;
                //dataArray[15] = c_in->zVelocity;
                //dataArray[16] = c_out->motor_torqueA;
                //dataArray[17] = c_out->motor_torqueB;
                //dataArray[18] = c_in->motor_currentA;
                //dataArray[19] = c_in->motor_currentB;
                //dataArray[20] = c_in->toe_switch;
                //dataArray[21] = c_in->command;

                i = (i++) % SHM_TO_USPACE_ENTRIES;
                dIndex++;
            }
            else {
                dIndex = 0;
                //for (i=0; i<100; i++) {
                //    dataClump[i] = 0;
                //}
            }
        }

        data_publisher.publish(dataClump);
        ros::spinOnce();
        loop_rate.sleep();
    }

    rt_shm_free(nam2num(SHM_NAME));   // Disconnect from shm.
    return 0;
}

