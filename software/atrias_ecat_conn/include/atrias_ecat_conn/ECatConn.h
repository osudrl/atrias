#ifndef NOOPCONN_H
#define NOOPCONN_H

/** @file
  * @brief This is the main class for the no-op connector.
  * This cycles RT Ops at 1 kHz.
  */

// Orocos
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/os/TimeService.hpp>

// ROS
#include <std_msgs/Header.h>

#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/globals.h>

namespace atrias {

namespace noopConn {

class NoopConn : public RTT::TaskContext {
	private:
	/** @brief By calling this, we cycle RT Ops.
	  */
	RTT::OperationCaller<void(atrias_msgs::robot_state)>
		newStateCallback;
	
	/** @brief Lets us report events, such as a missed deadline.
	  */
	RTT::OperationCaller<void(controllerManager::RtOpsEvent event)>
		sendEvent;
	
	/** @brief Holds the robot state to be passed to RT Ops.
	  */
	atrias_msgs::robot_state robotState;
	
	/** @brief Used to detect missed deadlines.
	  */
	bool                     waitingForResponse;
	
	public:
		/** @brief Initializes the Noop Connector
		  * @param name The name for this component.
		  */
		NoopConn(std::string name);
		
		/** @brief Called by RT Ops w/ update controller torques.
		  * @param controller_output The new controller output.
		  */
		void sendControllerOutput(atrias_msgs::controller_output controller_output);
		
		/** @brief Configures this component.
		  * Run by Orocos.
		  */
		bool configureHook();
		
		/** @brief Runs RTOps.
		  * This is run at 1 kHz by Orocos.
		  */
		void updateHook();
};

}

}

#endif // NOOPCONN_H
