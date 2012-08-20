#ifndef NOOPCONNECTOR_H
#define NOOPCONNECTOR_H

/** @file
  * @brief This is used to run our system with neither EtherCAT nor a Simulation.
  */

#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>

#include <atrias_rt_ops/Communicator.h>
#include <atrias_rt_ops/RTOps.h>

// Orocos
#include <rtt/os/TimeService.hpp>
#include <rtt/os/MutexLock.hpp>

#include "robot_invariant_defs.h"

class NoopConnector : public Communicator {
	// This lets us call the RobotData callback.
	RTOps*      rtOps;
	
	/** @brief Used by breakLoop() to signal loop() to stop.
	  */
	bool        stop;
	
	public:
		/** @brief Initializes the do-nothing connector.
		  * Does nothing.
		  */
		NoopConnector(RTOps* rt_ops);
		
		/** @brief Does nothing.
		  * @return Always true, because it never fails ;)
		  */
		bool init();
		
		/** @brief Runs the controllers. Called cyclicly.
		  */
		void step();
};

#endif // NOOPCONNECTOR_H
