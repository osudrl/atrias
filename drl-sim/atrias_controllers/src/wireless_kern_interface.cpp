// Devin Koepl

#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <pthread.h>

#include <rtai_shm.h>
#include <rtai_nam2num.h>

#include <ros/ros.h>
//#include "std_msgs/String.h"   // Testing Publisher
//#include <sstream>             // Testing Publisher

#include <atrias_controllers/uspace_kern_shm.h>
#include <atrias_controllers/atrias_srv.h>

// Macros for
#define QUOTEME_(x) #x
#define QUOTEME(x) 	QUOTEME_(x)

//*****************************************************************************

// SHM interface to kernel.

static Shm * shm;

//*****************************************************************************

// Get control parameters from GUI over ROS service atrias_srv.
bool atrias_gui_callback(atrias_controllers::atrias_srv::Request &, atrias_controllers::atrias_srv::Response &);

void print_msg(int);
void * msg_task(void *);

void log_data_entry(FILE *, int);
void * datalogging_task(void *);

//void push_data_to_gui(std_msgs::String);
//void * data_pushing_task(void *);

//ros::Publisher rtai_controller_pub;
//ros::Rate loop_rate(10);

//*****************************************************************************

int main(int argc, char **argv)
{
    int i;

    pthread_t datalogging_thread;
    pthread_t msg_thread;
    //pthread_t data_pushing_thread;

    //*************************************************************************

    // Connect to kernel's shm.

	rt_allow_nonroot_hrt();   // Allow rt_shm_alloc to be used without root permissions.
    if (!(shm = (Shm *) rt_shm_alloc(nam2num(SHM_NAME), 0, USE_VMALLOC)))
        ROS_ERROR("rtai_malloc() data to user space failed (maybe /dev/rtai_shm is missing)!");

    ROS_INFO("Connected to SHM.");

    //*************************************************************************

    // Startup ROS and interface service.

    ros::init(argc, argv, "gui_interface");
    ros::NodeHandle nh;
    ros::ServiceServer gui_srv = nh.advertiseService("gui_interface_srv", atrias_gui_callback);

    ROS_INFO("Service advertised.");

    //*************************************************************************

	// Start datalogging Publisher.

	//rtai_controller_pub = nh.advertise<std_msgs::String>("chatter", 1000);

	//ROS_INFO("Data logging Publisher advertised.");

    //*************************************************************************

    // Create threads.

    ROS_INFO("Creating threads.");

    //pthread_create(&msg_thread, NULL, msg_task, NULL);
    //pthread_create(&datalogging_thread, NULL, datalogging_task, NULL);

    //*************************************************************************

    ros::spin();

    //*************************************************************************

    // Wait for threads to finish.

    ROS_INFO("Waiting for threads to finish.");

    //pthread_join(msg_thread, NULL);
    //pthread_join(datalogging_thread, NULL);

    //*************************************************************************

    // Disconnect from kernel's shm.

    rt_shm_free(nam2num(SHM_NAME));

    ROS_INFO("\nDisconnected from SHM.");

    //*************************************************************************

    return 0;
}

//*****************************************************************************

void print_msg(int i)
{
    switch (shm->msg_priority[i])
    {
        case NO_MSG:
            break;
        case INFO_MSG:
            ROS_INFO("%s", shm->msg[i]);
            break;
        case WARN_MSG:
            ROS_WARN("%s", shm->msg[i]);
            break;
        case ERROR_MSG:
            ROS_ERROR("%s", shm->msg[i]);
            break;
        default:
            break;
    }
}

//*****************************************************************************

void * msg_task(void * arg)
{
    int i = 0;

    while (ros::ok())
    {
        if (i != shm->msg_index)
        {
            print_msg(i);

            i = (++i) % SHM_TO_USPACE_MSGS;
        }

        pthread_yield();
        ros::spinOnce;
    }

    while (i != shm->msg_index)
    {
        print_msg(i);

        i = (++i) % SHM_TO_USPACE_MSGS;
    }

    pthread_exit(NULL);
}

//*****************************************************************************

// Log a data entry.

void log_data_entry(FILE * fp, int i)
{
    ControllerInput * c_in;
    ControllerOutput * c_out;

    c_in = &shm->controller_input[i];
    c_out = &shm->controller_output[i];

    // fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %u, %u, %u, %u\n", c_in->body_angle, c_in->motor_angleA, c_in->motor_angleB,
    //     c_in->leg_angleA, c_in->leg_angleB, c_in->body_ang_vel, c_in->motor_velocityA, c_in->motor_velocityB,
    //     c_in->leg_velocityA, c_in->leg_velocityB, c_in->xPosition, c_in->yPosition, c_in->zPosition, c_in->xVelocity,
    //     c_in->xVelocity, c_in->yVelocity, c_in->zVelocity, c_out->motor_torqueA, c_out->motor_torqueB, c_in->motor_currentA, c_in->motor_currentB,
    //     c_in->toe_switch, c_in->command);

	fprintf( fp, "%f, %f, %f, %f\n", c_in->motor_angleA, c_in->motor_angleB, c_in->leg_angleA, c_in->leg_angleB);
}

//*****************************************************************************

// Datalog while the robot is running.

void * datalogging_task(void * arg)
{
    int i = 0;

    FILE *fp = fopen(QUOTEME(LOG_FILENAME), "w");

    // This print cannot be a ROS_INFO, because of a problem in the ROS header file?
    //ROS_INFO( "Writing %u log entries to %s.", shm->io_index, QUOTEME(LOG_FILENAME) );

    // Create file header.
    // fprintf(fp, "Body Angle, Motor Angle A, Motor Angle B, Leg Angle A, Leg Angle B, Body Angular Velocity, Motor Velocity A, Motor Velocity B, Leg Velocity A, Leg Velocity B, Height, Horizontal Velocity, Vertical Velocity, Motor Torque A, Motor Torque B, Motor Current A, Motor Current B\n");

	fprintf( fp, "Motor Angle A, Motor Angle B, Leg Angle A, Leg Angle B\n");

    while (ros::ok())
    {
        if (i != shm->io_index)
        {
            log_data_entry(fp, i);

            i = (++i) % SHM_TO_USPACE_ENTRIES;
        }

        pthread_yield();
        ros::spinOnce;
    }

    while (i != shm->io_index)
    {
        log_data_entry(fp, i);

        i = (++i) % SHM_TO_USPACE_ENTRIES;
    }

    fclose(fp);

    pthread_exit(NULL);
}

/****************************************************************************/

// Push data to GUI.

//void push_data_to_gui (std_msgs::String someMsg) {
//    rtai_controller_pub.publish(someMsg);
//}
//
//void *data_pushing_task (void *) {
//    int count = 0;
//
//    while (ros::ok()) {
//        std_msgs::String msg;
//        std::stringstream ss;
//        ss << "hello world " << count;
//        msg.data = ss.str();
//    
//        ROS_INFO("%s", msg.data.c_str());
//    
//        push_data_to_gui(msg);
//    
//        pthread_yield();
//        ros::spinOnce();
//
//        //loop_rate.sleep();
//        count++;
//    }
//
//    pthread_exit(NULL);
//}

/*
// Print any messge waiting from the RT thread.
void rt_msg_print( void )
{
    switch ( rt_msg_priority )
    {
        case NO_MSG:
            break;
        case INFO_MSG:
            ROS_INFO( "%s", rt_msg );
            break;
        case WARN_MSG:
            ROS_WARN( "%s", rt_msg );
            break;
        case ERROR_MSG:
            ROS_ERROR( "%s", rt_msg );
    }

    // Clear the message indicator.
    rt_msg_priority = NO_MSG;
}
 */

//*****************************************************************************

bool atrias_gui_callback(atrias_controllers::atrias_srv::Request &req, atrias_controllers::atrias_srv::Response &res)
{
    int i;

    //*************************************************************************

    // Load request into the kernel buffer.

    shm->controller_data[1 - shm->control_index].command = req.command;
    shm->controller_data[1 - shm->control_index].controller_requested = req.controller_requested;

    for (i = 0; i < SIZE_OF_CONTROLLER_DATA; i++)
        shm->controller_data[1 - shm->control_index].data[i] = req.control_data[i];

    shm->req_switch = true;

    //*************************************************************************

    // Populate response from userspace buffer.

    res.body_angle      = shm->controller_input[shm->io_index - 1].body_angle;
    res.motor_angleA    = shm->controller_input[shm->io_index - 1].motor_angleA;
    res.motor_angleB    = shm->controller_input[shm->io_index - 1].motor_angleB;
    res.leg_angleA      = shm->controller_input[shm->io_index - 1].leg_angleA;
    res.leg_angleB      = shm->controller_input[shm->io_index - 1].leg_angleB;
    res.xPosition       = shm->controller_input[shm->io_index - 1].xPosition;
    res.yPosition       = shm->controller_input[shm->io_index - 1].yPosition;
    res.zPosition       = shm->controller_input[shm->io_index - 1].zPosition;
    res.xVelocity       = shm->controller_input[shm->io_index - 1].xVelocity;
    res.yVelocity       = shm->controller_input[shm->io_index - 1].yVelocity;
    res.zVelocity       = shm->controller_input[shm->io_index - 1].zVelocity;
    res.thermistorA[0]  = shm->controller_input[shm->io_index - 1].thermistorA[0];
    res.thermistorA[1]  = shm->controller_input[shm->io_index - 1].thermistorA[1];
    res.thermistorA[2]  = shm->controller_input[shm->io_index - 1].thermistorA[2];
    res.thermistorB[0]  = shm->controller_input[shm->io_index - 1].thermistorB[0];
    res.thermistorB[1]  = shm->controller_input[shm->io_index - 1].thermistorB[1];
    res.thermistorB[2]  = shm->controller_input[shm->io_index - 1].thermistorB[2];
    res.motorVoltageA   = shm->controller_input[shm->io_index - 1].motorVoltageA;
    res.motorVoltageB   = shm->controller_input[shm->io_index - 1].motorVoltageB;
    res.logicVoltageA   = shm->controller_input[shm->io_index - 1].logicVoltageA;
    res.logicVoltageB   = shm->controller_input[shm->io_index - 1].logicVoltageB;
    res.medullaStatusA  = shm->controller_input[shm->io_index - 1].medullaStatusA;
    res.medullaStatusB  = shm->controller_input[shm->io_index - 1].medullaStatusB;

    res.motor_torqueA   = shm->controller_output[shm->io_index - 1].motor_torqueA;
    res.motor_torqueB   = shm->controller_output[shm->io_index - 1].motor_torqueB;
    res.motor_torque_hip= shm->controller_output[shm->io_index - 1].motor_torque_hip;

    res.motor_velocityA   = shm->controller_input[shm->io_index - 1].motor_velocityA;
    res.motor_velocityB   = shm->controller_input[shm->io_index - 1].motor_velocityB;
    res.motor_velocity_hip= shm->controller_input[shm->io_index - 1].hip_angle_vel;
    
    for (i = 0; i < SIZE_OF_CONTROLLER_STATE_DATA; i++)
        res.control_state[i] = shm->controller_state.data[i];

    //*************************************************************************

    return true;
}
