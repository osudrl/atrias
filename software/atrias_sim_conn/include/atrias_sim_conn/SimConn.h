#ifndef SIMCONN_H
#define SIMCONN_H

/** @file
  * @brief This is the main class for the simulation connector.
  */

// Orocos
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>

// ROS


#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/globals.h>

namespace atrias {

namespace simConn {

class SimConn : public RTT::TaskContext {
	private:
	/** @brief By calling this, we cycle RT Ops.
	  */
	RTT::OperationCaller<void(atrias_msgs::robot_state)>
		newStateCallback;
	
	public:
		/** @brief Initializes the Sim Connector
		  * @param name The name for this component.
		  */
		SimConn(std::string name);
		
		/** @brief Called by RT Ops w/ update controller torques.
		  * @param controller_output The new controller output.
		  */
		void sendControllerOutput(atrias_msgs::controller_output controller_output);
		
		/** @brief Configures this component.
		  * Run by Orocos.
		  */
		bool configureHook();
};

}

}

#endif // SIMCONN_H
