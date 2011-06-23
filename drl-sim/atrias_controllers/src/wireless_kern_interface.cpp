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

#define INTERFACE_BUFFER_SIZE 10000000

//*****************************************************************************

// SHM interface to kernel.

ToKernShm		* to_kern_shm;
ToUspaceShm		* to_uspace_shm;

//*****************************************************************************

bool atrias_gui_callback(atrias_controllers::atrias_srv::Request &, atrias_controllers::atrias_srv::Response &);
void * datalogging_task( void * );

int main (int argc, char **argv)
{
	int i;

	//pthread_t datalogging_thread;

	//*************************************************************************

	// Connect to kernel's shm.

	to_kern_shm = ( ToKernShm * )rtai_malloc( nam2num( "SHM_TO_KERN_NAM" ), 0 );
	if ( to_kern_shm == NULL ) 
		ROS_ERROR( "rtai_malloc() data to kernel failed (maybe /dev/rtai_shm is missing)!" );

	to_uspace_shm = ( ToUspaceShm * )rtai_malloc( nam2num( "SHM_TO_USPACE_NAM" ), 0 );
	if ( to_uspace_shm == NULL ) 
		ROS_ERROR( "rtai_malloc() data to user space failed (maybe /dev/rtai_shm is missing)!" );

	ROS_INFO( "Connected to SHM." );

	//*************************************************************************

	// Startup ROS and interface service.

	ros::init(argc, argv, "gui_interface");
	ros::NodeHandle nh;
	ros::ServiceServer gui_srv = nh.advertiseService("gui_interface_srv", atrias_gui_callback);

	ROS_INFO("Service advertised.");

	//*************************************************************************

	// Create datalogging thread.
	//pthread_create(&datalogging_thread, NULL, datalogging_task, NULL);	

	while ( ros::ok() )
	{
		ros::spinOnce();
	}

	//*************************************************************************

	// Write the log file.

	/*ROS_INFO( "\nWriting %u log file entries to %s.", uspace_buffer_index, QUOTEME(LOG_FILENAME) );

	for ( i = 0; i < uspace_buffer_index; i++)
	{
		// Log the data.
		data_logger.log( &to_uspace_buffer[i] );		
	}*/

	//*************************************************************************

	// Disconnect from kernel's shm.

	rt_shm_free( nam2num( "SHM_TO_KERN_NAM" ) );
	rt_shm_free( nam2num( "SHM_TO_USPACE_NAM" ) );

	ROS_INFO("Disconnected from SHM.");

	//*************************************************************************

	return 0;
}

//*****************************************************************************

/*void * datalogging_task( void * argument )
{
	int i;

	while ( ros::ok() )
	{
		// Check to see if there is fresh data in the ring buffer.
		while ( to_uspace_shm[to_uspace_index]->fresh )
		{
			// Load the userspace buffer with the most recent sensor information from the robot.
			to_uspace_buffer[uspace_buffer_index] = *to_uspace_shm[to_uspace_index];
			to_uspace_shm[to_uspace_index]->fresh = false;

			//if ( to_uspace_buffer[uspace_buffer_index].controller_input.leg_angleA == 0 )
			//	ROS_WARN( "Bad read at index %u.", to_uspace_index );

			// Increment and roll the ring buffer index over, when it reaches the end of the buffer.
			to_uspace_index = (++to_uspace_index) % SHM_TO_USPACE_ENTRIES;
			uspace_buffer_index = (++uspace_buffer_index) % INTERFACE_BUFFER_SIZE;
		}

		//pthread_yield();
	}

	return NULL;
}*/

//*****************************************************************************

bool atrias_gui_callback(atrias_controllers::atrias_srv::Request &req, atrias_controllers::atrias_srv::Response &res)
{
	int i;

	//*************************************************************************

	// Load request into the kernel buffer.

	to_kern_shm->command[to_kern_shm->index ^ 1]				= req.command;
	to_kern_shm->controller_requested[to_kern_shm->index ^ 1]	= req.controller_requested;

	for (i = 0; i < SIZE_OF_CONTROLLER_DATA; i++)
	{
		to_kern_shm->controller_data[to_kern_shm->index ^ 1][i] = req.control_data[i];
	}

	to_kern_shm->req_switch										= true;

	//*************************************************************************

	// Populate response from userspace buffer.
	
	res.body_angle		= to_uspace_shm->controller_input[to_uspace_shm->index].body_angle;
	res.motor_angleA	= to_uspace_shm->controller_input[to_uspace_shm->index].motor_angleA;
	res.motor_angleB	= to_uspace_shm->controller_input[to_uspace_shm->index].motor_angleB;
	res.leg_angleA		= to_uspace_shm->controller_input[to_uspace_shm->index].leg_angleA;
	res.leg_angleB		= to_uspace_shm->controller_input[to_uspace_shm->index].leg_angleB;
	res.hor_vel			= to_uspace_shm->controller_input[to_uspace_shm->index].horizontal_velocity;
	res.height			= to_uspace_shm->controller_input[to_uspace_shm->index].height;

	res.motor_torqueA	= to_uspace_shm->controller_output[to_uspace_shm->index].motor_torqueA;
	res.motor_torqueB	= to_uspace_shm->controller_output[to_uspace_shm->index].motor_torqueB;

	for (i = 0; i < SIZE_OF_CONTROLLER_STATE_DATA; i++)
	{
		res.control_state[i] = to_uspace_shm->controller_state.data[i];
	}

	//*************************************************************************

	return true;
}
