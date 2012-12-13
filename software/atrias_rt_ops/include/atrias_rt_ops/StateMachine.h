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
	/** @brief Counts up while a reset happens. Used to detect a failed reset.
	  */
	int                             resetCounter;
	
	/** @brief Points to the main RT Ops class.
	  */
	RTOps*                          rtOps;
	
	/** @brief The current state.
	  */
	RtOpsState                      currentState;
	
	/** @brief Protects the state from concurrent access.
	  */
	RTT::os::Mutex                  currentStateLock;
	
	/** @brief Sets the state.
	  * @param new_state The new state.
	  */
	void                            setState(RtOpsState new_state);
	
	/** @brief Responds to the controller manager.
	  * @param state The state for which to send an acknowledgement.
	  * This sends out an event to inform the controller manager that RT Ops has
	  * finished processing its command.
	  */
	void                            ackCMState(RtOpsState state);
	
	public:
		/** @brief Initializes this StateMachine.
		  * @param rt_ops A pointer to the RT Ops class.
		  */
		StateMachine(RTOps* rt_ops);
		
		/** @brief Executes an EStop.
		  * @param event The reason for this EStop.
		  * This sets our state to ESTOP, resulting in an estop on the Medullas
		  * (if possible). This also sends out an RtOpsEvent with the reason for this
		  * estop.
		  */
		void eStop(RtOpsEvent event);
		
		/** @brief Computes a new state.
		  * @return The new desired Medulla state.
		  */
		medulla_state_t calcState(atrias_msgs::controller_output controllerOutput);
		
		/** @brief Sets a new state for the state machine.
		  * @param new_state The new state.
		  */
		void newCMState(RtOpsState new_state);
		
		/** @brief Gets the current RT Ops state.
		  * @return The current RT Ops state.
		  */
		RtOpsState getRtOpsState();
};

}

}

#endif // STATEMACHINE_H

// vim: noexpandtab
