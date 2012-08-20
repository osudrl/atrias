#ifndef GAZEBOCONNECTOR_H
#define GAZEBOCONNECTOR_H

/** @file
  * @brief Connects with the simulation.
  */

#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>

#include <atrias_rt_ops/Communicator.h>
#include <atrias_rt_ops/RTOps.h>

#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>

#include <ros/ros.h>
#include <rtt/os/MutexLock.hpp>

class GazeboConnector : public Communicator {
	// This lets us access the robot state and call the RobotData callback.
	RTOps*      rtOps;
	
	/** @brief Holds whether or not the GUI has commanded an enable.
	  */
	bool        guiEnabled;
	
	/** @brief Contains the eStop status.
	  */
	bool        eStopped;
	
	public:
		GazeboConnector(RTOps* rt_ops);
		
		~GazeboConnector(void);
		
		bool init();
		
		void transmitHook();
		
		void disable();
		
		void enable();
		
		void eStop();
		
		void leaveEStop();
		
	private:
		atrias_msgs::controller_output cosi;  // Controller out, simulation in
};

#endif // GAZEBOCONNECTOR_H
