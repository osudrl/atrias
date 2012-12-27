#ifndef STATEMACHINE_H
#define STATEMACHINE_H

/** @file
  * @brief Contains the code that processes the Controller Manager's state
  * machine command and outputs a desired medulla state.
  */

class StateMachine;

// Orocos
#include <rtt/os/Mutex.hpp>
#include <rtt/os/MutexLock.hpp>

#include <atrias_shared/globals.h>
#include <robot_invariant_defs.h>
#include <atrias_msgs/controller_output.h>

#include "atrias_rt_ops/RTOps.h"

namespace atrias {

namespace rtOps {

class StateMachine {
	/** @brief This is the last command from the controller
	  */
	RtOpsState     controllerCmd;

	/** @brief Points to the main RT Ops class.
	  */
	RTOps*         rtOps;
	
	/** @brief The current state.
	  */
	RtOpsState     state;

	/** @brief This protects access to the current state.
	  */
	RTT::os::Mutex stateLock;

	/** @brief This handles everything common to each "running" state.
	  * @return Whether or not this has triggered a state change.
	  */
	bool runningCommon(RtOpsState guiCmd);
	
	/** @brief Sets the state.
	  * @param new_state The new state.
	  * @param event     The event to send out with the new state info.
	  */
	void setState(RtOpsState new_state, atrias_msgs::rt_ops_event event);
	
	public:
		/** @brief Initializes this StateMachine.
		  * @param rt_ops A pointer to the RT Ops class.
		  */
		StateMachine(RTOps* rt_ops);

		/** @brief Calculates the command to be sent to the Medullas
		  * @param controller_cmd The controller's "command"
		  * @return The command for the medullas
		  */
		medulla_state_t calcMedullaCmd(RtOpsState controller_cmd);
		
		/** @brief Computes a new state.
		  * This MUST be called from the controller thread!
		  */
		void run();
		
		/** @brief Gets the current RT Ops state.
		  * @return The current RT Ops state.
		  */
		RtOpsState getState();
};

}

}

#endif // STATEMACHINE_H

// vim: noexpandtab
