#ifndef ELABSCONN_H
#define ELABSCONN_H

/** @file
  * @brief This is the main class for the EtherLabs connector.
  * This connector handles communicating with the robot and with the simulation
  * over EtherCAT w/ EtherLabs.
  */

namespace atrias {
namespace ecatConn{
class ECatConn;
}}

// Orocos
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Logger.hpp>

// ROS
#include <std_msgs/Header.h>

#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/globals.h>

#include "atrias_elabs_conn/ConnManager.h"
#include "atrias_elabs_conn/MedullaManager.h"

namespace atrias {

namespace elabsConn {

class ELabsConn : public RTT::TaskContext {
	/** @brief Handles the main operation of this component.
	  */
	ConnManager*   connManager;
	
	/** @brief Handles all our medulla objects.
	  */
	MedullaManager medullaManager;
	
	public:
		/** @brief Initializes this Connector
		  * @param name The name for this component.
		  */
		ELabsConn(std::string name);
		
		/** @brief By calling this, we cycle RT Ops.
		  */
		RTT::OperationCaller<void(atrias_msgs::robot_state)>
			newStateCallback;
		
		/** @brief Called by RT Ops w/ updated controller torques.
		  * @param controller_output The new controller output.
		  */
		void sendControllerOutput(atrias_msgs::controller_output controller_output);
		
		/** @brief Lets us report events, such as a missed deadline.
		  */
		RTT::OperationCaller<void(controllerManager::RtOpsEvent event)>
			sendEvent;
		
		/** @brief Configures this component.
		  * Run by Orocos.
		  * @return Success.
		  */
		bool configureHook();
		
		/** @brief Starts execution of this component.
		  * Run by Orocos.
		  * @return Success.
		  */
		bool startHook();
		
		/** @brief Stops execution of this component.
		  * Run by Orocos.
		  * @return Success.
		  */
		void stopHook();
		
		/** @brief Lets ConnManager access our \a MedullaManager
		  * @return A pointer to the MedullaManager.
		  */
		MedullaManager* getMedullaManager();
};

}

}

#endif // ELABSCONN_H
