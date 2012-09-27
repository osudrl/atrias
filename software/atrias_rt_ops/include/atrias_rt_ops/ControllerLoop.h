#ifndef CONTROLLERLOOP_H
#define CONTROLLERLOOP_H

/** @file
  * @brief Contains the main loop for the controllers.
  */

class ControllerLoop;

// Orocos
#include <rtt/Activity.hpp>
#include <rtt/os/Semaphore.hpp>
#include <rtt/os/Mutex.hpp>
#include <rtt/os/MutexLock.hpp>
#include <rtt/Logger.hpp>

// ATRIAS
#include <robot_invariant_defs.h>
#include <robot_variant_defs.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/drl_math.h>

#include "atrias_rt_ops/RTOps.h"
#include "atrias_rt_ops/StateMachine.h"

namespace atrias {

namespace rtOps {

class ControllerLoop : public RTT::Activity {
	/** @brief A pointer to RT Ops to we can access its methods.
	  */
	RTOps*             rtOps;
	
	/** @brief Used by \a breakLoop() to signal \a loop() to exit.
	  */
	volatile bool      done;
	
	/** @brief Used to signal the control loop to run.
	  */
	RTT::os::Semaphore signal;
	
	/** @brief Protects access to \a signal
	  */
	RTT::os::Mutex     signalLock;
	
	/** @brief Prevents concurrency issues during controller unloading.
	  */
	RTT::os::Mutex     controllerLock;
	
	/** @brief Stores whether or not a controller is loaded.
	  */
	volatile bool      controllerLoaded;
	
	/** @brief Clamps the controller output.
	  * @param controller_output The un-clamped outputs.
	  * @return The clamped outputs.
	  */
	atrias_msgs::controller_output
		clampControllerOutput(atrias_msgs::controller_output controller_output);
	
	public:
		/** @brief Initializes this ControllerLoop.
		  */
		ControllerLoop(RTOps* rt_ops);
		
		/** @brief Tells the controller loop a controller is loaded.
		  * This function may be called repeatedly with no ill effects.
		  */
		void setControllerLoaded();
		
		/** @brief Tells the controller loop that no controller is loaded.
		  * This function will wait until it is safe to unload the controller.
		  */
		void setControllerUnloaded();
		
		/** @brief Run by Orocos. Is the main loop for the controllers..
		  */
		void loop();
		
		/** @brief Instructs the loop to run once.
		  */
		void cycleLoop();
		
		/** @brief Run by Orocos to shut down the main loop.
		  * @return Success.
		  */
		bool breakLoop();
		
		/** @brief Initializes the ControllerLoop. Run before \a loop().
		  * @return Success.
		  */
		bool initialize();
};

}

}

#endif
