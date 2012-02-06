#include <atrias_controllers/data_publisher.h>

void callbackShm (const ros::TimerEvent&) {
    while (i != shm->io_index) {   // Check if shm has been updated.
        c_in = &shm->controller_input[i];
        c_out = &shm->controller_output[i];

        // When adding new fields, make sure to update AtriasData.msg.
        aData.time = i;
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

        // Publish the data. This takes up the majority of the loop time.
        data_publisher.publish(aData);

        i = (i++) % SHM_TO_USPACE_ENTRIES;
        ROS_INFO("%d %d", i, shm->io_index);
    }
        ROS_INFO("shm not updated yet!");
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "data_publisher");

    // Initialize the nodehandle. This must be called after ros::init.
    ros::NodeHandle nh;

    // Advertise the Publisher.
    //
    // atrias_controllers::AtriasData - Format of ROS msg to publish.
    // "datalog_downlink" - Name of topic to publish. ROS Subscribers can
    //                      listen to what this Publisher publishes by
    //                      subscribing to this topic.
    // 1000 - Number of ROS msgs to queue up before dropping the oldest ones.
    data_publisher = nh.advertise<atrias_msgs::atrias_data>("datalog_downlink", 0);

    // Initialize the Timer. (NOTE: This is NOT a replacement for realtime.
    // There are no guarantees about how accurate this is.)
    //
    // ros::Duration - ROS primitive type representing a period of Time.
    //                 i.e., Time is a specific moment (e.g., 5:00 pm) whereas
    //                 Duration is a period of Time (e.g., 5 hours).
    // 0.002 - 2 milliseconds. The timer duration shouldn't be too critical, as
    //         callbackShm uses a while (not if) loop to check for io_index
    //         updates.
    // callbackShm - function to call every duration..
    timer = nh.createTimer(ros::Duration(0.002), callbackShm);

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
        
    ROS_INFO("Datalog publisher connected to shm.");

    // Set i to current value of io_index.
    i = shm->io_index;

    ros::spin();

    // Disconnect from shm.
    rt_shm_free(nam2num(SHM_NAME));

    return 0;
}

