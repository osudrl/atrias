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

#include <atrias_controllers/uspace_kern_shm.h>
#include <atrias_controllers/atrias_srv.h>

// Macros for
#define QUOTEME_(x) #x
#define QUOTEME(x) 	QUOTEME_(x)

//*****************************************************************************

// SHM interface to kernel.

static Shm * shm;

//*****************************************************************************

bool atrias_gui_callback(atrias_controllers::atrias_srv::Request &, atrias_controllers::atrias_srv::Response &);

void print_msg( int );
void * msg_task( void * );

void log_data_entry( FILE *, int );
void * datalogging_task( void * );

//*****************************************************************************

int main (int argc, char **argv)
{
	int i;

	pthread_t datalogging_thread;
	pthread_t msg_thread;

	//*************************************************************************

	// Connect to kernel's shm.

	if ( !( shm = ( Shm * )rt_shm_alloc( nam2num( SHM_NAME ), 0, USE_VMALLOC ) ) )
		ROS_ERROR( "rtai_malloc() data to user space failed (maybe /dev/rtai_shm is missing)!" );

	ROS_INFO( "Connected to SHM." );

	//*************************************************************************

	// Startup ROS and interface service.

	ros::init(argc, argv, "gui_interface");
	ros::NodeHandle nh;
	ros::ServiceServer gui_srv = nh.advertiseService("gui_interface_srv", atrias_gui_callback);

	ROS_INFO("Service advertised.");

	//*************************************************************************

	// Create threads.

	ROS_INFO( "Creating threads." );

	pthread_create( &msg_thread, NULL, msg_task, NULL );
	pthread_create( &datalogging_thread, NULL, datalogging_task, NULL );	

	//*************************************************************************

	ros::spin();

	//*************************************************************************

	// Wait for threads to finish.

	ROS_INFO( "Waiting for threads to finish." );

	pthread_join( msg_thread, NULL );
	pthread_join( datalogging_thread, NULL );

	//*************************************************************************

	// Disconnect from kernel's shm.

	rt_shm_free( nam2num( SHM_NAME ) );

	ROS_INFO("\nDisconnected from SHM.");

	//*************************************************************************

	return 0;
}

//*****************************************************************************

void print_msg( int i )
{
	switch ( shm->msg_priority[i] )
	{
		case NO_MSG:
			break;
		case INFO_MSG:
			ROS_INFO( "%s", shm->msg[i] );
			break;
		case WARN_MSG:
			ROS_WARN( "%s", shm->msg[i] );
			break;
		case ERROR_MSG:
			ROS_ERROR( "%s", shm->msg[i] );
			break;
		default:
			break;
	}
}

//*****************************************************************************

void * msg_task( void * arg )
{
	int i = 0;

	while ( ros::ok() )
	{
		if ( i != shm->msg_index )
		{
			print_msg( i );
		
			i = (++i) % SHM_TO_USPACE_MSGS;
		}	

		pthread_yield();
		ros::spinOnce;
	}

	while ( i != shm->msg_index )
	{
		print_msg( i );
		
		i = (++i) % SHM_TO_USPACE_MSGS;
	}

	pthread_exit( NULL );
}

//*****************************************************************************

// Log a data entry.

void log_data_entry( FILE * fp, int i )
{
	ControllerInput * c_in;
	ControllerOutput * c_out;

	c_in	= &shm->controller_input[i];
	c_out 	= &shm->controller_output[i];

	fprintf( fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %u, %u\n", c_in->body_angle, c_in->motor_angleA, c_in->motor_angleB,
		c_in->leg_angleA, c_in->leg_angleB, c_in->body_ang_vel, c_in->motor_velocityA, c_in->motor_velocityB,
		c_in->leg_velocityA, c_in->leg_velocityB, c_in->height, c_in->horizontal_velocity, c_in->vertical_velocity,
		c_out->motor_torqueA, c_out->motor_torqueB, c_in->motor_currentA, c_in->motor_currentB );
}

//*****************************************************************************

// Datalog while the robot is running.

void * datalogging_task( void * arg )
{
	int i = 0;

	FILE *fp = fopen( QUOTEME(LOG_FILENAME), "w" );

	// This print cannot be a ROS_INFO, because of a problem in the ROS header file?
	//ROS_INFO( "Writing %u log entries to %s.", shm->io_index, QUOTEME(LOG_FILENAME) );

	// Create file header.
	fprintf( fp, "Body Angle, Motor Angle A, Motor Angle B, Leg Angle A, Leg Angle B, Body Angular Velocity, Motor Velocity A, Motor Velocity B, Leg Velocity A, Leg Velocity B, Height, Horizontal Velocity, Vertical Velocity, Motor Torque A, Motor Torque B, Motor Current A, Motor Current B\n");		

	while ( ros::ok() )
	{
		if ( i != shm->io_index )
		{
			log_data_entry( fp, i );
			
			i = (++i) % SHM_TO_USPACE_ENTRIES;
		}

		pthread_yield();
		ros::spinOnce;
	}

	while ( i != shm->io_index )
	{
		log_data_entry( fp, i );
		
		i = (++i) % SHM_TO_USPACE_ENTRIES;
	}

	fclose(fp);

	pthread_exit( NULL );
}

/****************************************************************************/

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

	shm->command[1 - shm->control_index]				= req.command;
	shm->controller_requested[1 - shm->control_index]	= req.controller_requested;

	for (i = 0; i < SIZE_OF_CONTROLLER_DATA; i++)
	{
		shm->controller_data[1 - shm->control_index][i] = req.control_data[i];
	}

	shm->req_switch										= true;

	//*************************************************************************

	// Populate response from userspace buffer.
	
	res.body_angle		= shm->controller_input[shm->io_index - 1].body_angle;
	res.motor_angleA	= shm->controller_input[shm->io_index - 1].motor_angleA;
	res.motor_angleB	= shm->controller_input[shm->io_index - 1].motor_angleB;
	res.leg_angleA		= shm->controller_input[shm->io_index - 1].leg_angleA;
	res.leg_angleB		= shm->controller_input[shm->io_index - 1].leg_angleB;
	res.hor_vel			= shm->controller_input[shm->io_index - 1].horizontal_velocity;
	res.height			= shm->controller_input[shm->io_index - 1].height;

	res.motor_torqueA	= shm->controller_output[shm->io_index - 1].motor_torqueA;
	res.motor_torqueB	= shm->controller_output[shm->io_index - 1].motor_torqueB;

	for (i = 0; i < SIZE_OF_CONTROLLER_STATE_DATA; i++)
	{
		res.control_state[i] = shm->controller_state.data[i];
	}

	//*************************************************************************

	return true;
}
