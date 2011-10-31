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

    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;

        ControllerInput *c_in;
        if (i != shm->io_index) {
            c_in = &shm->controller_input[i];
            i = (i++) % SHM_TO_USPACE_ENTRIES;
        }

        ss << c_in->motor_angleA;
        msg.data = ss.str();

        //ROS_INFO("%s", msg.data.c_str());

        atrias_datalog_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    rt_shm_free(nam2num(SHM_NAME));   // Disconnect from shm.
    return 0;
}

