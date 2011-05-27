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
#include <atrias_controllers/data_logger.h>

// Macros for
#define QUOTEME_(x) #x
#define QUOTEME(x) 	QUOTEME_(x)

//*****************************************************************************

// SHM interface to kernel.

int				to_uspace_index = 0;

DataToKern		*to_kern_shm;
DataToUspace	*to_uspace_shm;

DataToKern		to_kern_buffer;
DataToUspace	to_uspace_buffer;

//*****************************************************************************

bool atrias_gui_callback(atrias_controllers::atrias_srv::Request &, atrias_controllers::atrias_srv::Response &);
void * datalogging_task( void * );

int main (int argc, char **argv)
{
	pthread_t datalogging_thread;

	//*************************************************************************

	// Connect to kernel's shm.

	to_kern_shm = (DataToKern *)rtai_malloc(nam2num("SHM_TO_KERN_NAM"), 0);
	to_kern_shm = (DataToKern *)rtai_malloc(nam2num("SHM_TO_KERN_NAM"), 0);
	to_kern_shm = (DataToKern *)rtai_malloc(nam2num("SHM_TO_KERN_NAM"), 0);
	to_kern_shm = (DataToKern *)rtai_malloc(nam2num("SHM_TO_KERN_NAM"), 0);
	if (to_kern_shm == NULL) 
	{
		ROS_ERROR("rtai_malloc() data to kernel failed (maybe /dev/rtai_shm is missing)!");
	}

	to_uspace_shm = (DataToUspace *)rtai_malloc(nam2num("SHM_TO_USPACE_NAM"), 0);
	to_uspace_shm = (DataToUspace *)rtai_malloc(nam2num("SHM_TO_USPACE_NAM"), 0);
	to_uspace_shm = (DataToUspace *)rtai_malloc(nam2num("SHM_TO_USPACE_NAM"), 0);
	to_uspace_shm = (DataToUspace *)rtai_malloc(nam2num("SHM_TO_USPACE_NAM"), 0);
	if (to_uspace_shm == NULL) 
	{
		ROS_ERROR("rtai_malloc() data to user space failed (maybe /dev/rtai_shm is missing)!");
	}

	ROS_INFO("Connected to SHM.");

	//*************************************************************************

	// Startup ROS and interface service.

	ros::init(argc, argv, "gui_interface");
	ros::NodeHandle nh;
	ros::ServiceServer gui_srv = nh.advertiseService("gui_interface_srv", atrias_gui_callback);

	ROS_INFO("Service advertised.");

	//*************************************************************************

	// Datalog at 1kHz.

	ROS_INFO("Log Filename: %s", QUOTEME(LOG_FILENAME));

	// Create datalogging thread.
	pthread_create(&datalogging_thread, NULL, datalogging_task, NULL);

	while ( ros::ok() )
	{
		ros::spinOnce();
	}

	//*************************************************************************

	// Disconnect from kernel's shm.

	rtai_free (nam2num("SHM_TO_KERN_NAM"), shm);
	rtai_free (nam2num("SHM_TO_KERN_NAM"), shm);
	rtai_free (nam2num("SHM_TO_KERN_NAM"), shm);
	rtai_free (nam2num("SHM_TO_KERN_NAM"), shm);

	rtai_free (nam2num("SHM_TO_USPACE_NAM"), shm);
	rtai_free (nam2num("SHM_TO_USPACE_NAM"), shm);
	rtai_free (nam2num("SHM_TO_USPACE_NAM"), shm);
	rtai_free (nam2num("SHM_TO_USPACE_NAM"), shm);

	ROS_INFO("Disconnected from SHM.");

	//*************************************************************************

	return 0;
}

bool atrias_gui_callback(atrias_controllers::atrias_srv::Request &req, atrias_controllers::atrias_srv::Response &res)
{
	int i;

	//*************************************************************************

	// Load request into the kernel buffer.

	to_kern_buffer.command					= req.command;
	to_kern_buffer.controller_requested		= req.controller_requested;

	for (i = 0; i < SIZE_OF_CONTROLLER_DATA; i++)
	{
		to_kern_buffer.controller_data[i] = req.control_data[i];
	}

	//*************************************************************************

	// Populate response from userspace buffer.

	res.body_angle		= to_uspace_buffer.controller_input.body_angle;
	res.motor_angleA	= to_uspace_buffer.controller_input.motor_angleA;
	res.motor_angleB	= to_uspace_buffer.controller_input.motor_angleB;
	res.leg_angleA		= to_uspace_buffer.controller_input.leg_angleA;
	res.leg_angleB		= to_uspace_buffer.controller_input.leg_angleB;
	res.hor_vel			= to_uspace_buffer.controller_input.horizontal_velocity;
	res.height			= to_uspace_buffer.controller_input.height;

	res.motor_torqueA	= to_uspace_buffer.controller_output.motor_torqueA;
	res.motor_torqueB	= to_uspace_buffer.controller_output.motor_torqueB;

	//*************************************************************************

	return true;
}

void * datalogging_task( void * argument )
{
	int i;

	DataLogger	data_logger = DataLogger(QUOTEME(LOG_FILENAME));

	while ( ros::ok() )
	{
		// Check to see if there is fresh data in the ring buffer.
		if ( to_uspace_shm[to_uspace_index].fresh )
		{
			// Load the userspace buffer with the most recent sensor information from the robot.
			to_uspace_buffer = to_uspace_shm[to_uspace_index];

			// Log the new data entry.
			data_logger.log( &to_uspace_buffer );

			// Increment and roll the ring buffer index over, when it reaches the end of the buffer.
			to_uspace_index = (++to_uspace_index) % SHM_TO_USPACE_ENTRIES;

			to_uspace_shm[to_uspace_index].fresh = false;
		}

		// If the kernel shm is free, then load the kernel buffer.
		if ( !to_kern_shm->lock )
		{
			to_kern_shm->lock = true;

			to_kern_shm->command 				= to_kern_buffer.command;
			to_kern_shm->controller_requested	= to_kern_buffer.controller_requested;
			
			for (i = 0; i < SIZE_OF_CONTROLLER_DATA; i++)
			{
				to_kern_shm->controller_data[i] = to_kern_buffer.controller_data[i];
			}

			to_kern_shm->lock = false;
		}

		pthread_yield();
	}

	return NULL;
}
