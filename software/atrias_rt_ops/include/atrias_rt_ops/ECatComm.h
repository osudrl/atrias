#ifndef ECATCOMM_H
#define ECATCOMM_H

/** @file
  * @brief Interfaces between the Orocos stuff and the EtherCAT Master.
  */

class ECatComm;

#include <stdint.h>
extern "C" {
#include <ethercattype.h>
#include <ethercatmain.h>
#include <ethercatconfig.h>
#include <ethercatdc.h>
}
#include <stdio.h>
#include <time.h>

// Orocos
#include <rtt/os/TimeService.hpp>
#include <rtt/os/Mutex.hpp>
#include <rtt/os/MutexLock.hpp>

#include "robot_invariant_defs.h"
#include "atrias_rt_ops/RTOps.h"
#include "atrias_rt_ops/ECatSlavesManager.h"
#include "atrias_rt_ops/MedullaManager.h"
#include "atrias_rt_ops/Communicator.h"

#define EC_TIMEOUT_US      500
// The gain for the overshoot-compensation filter
#define TIMING_FILTER_GAIN 100

//! @brief Performs the actual interaction with the EtherCAT Master
class ECatComm : public Communicator {
	// What does this do? It's in simple_test... but the Doxygen docs don't really help :(
	char                        IOmap[4096];
	
	/** @brief Holds the ECatSlavesManager.
	  */
	ECatSlavesManager*          slavesManager;
	
	/** @brief Stores a pointer to the RT Operations instance for access to
	  * robot state, controller outputs, and the robot state update function.
	  */
	RTOps*                      rtOps;
	
	/** @brief Used to instruct loop() that it needs to stop.
	  */
	volatile bool               done;
	
	
	RTT::os::TimeService::nsecs getNSecs();
	
	/** @brief Protects access to SOEM.
	  * SOEM itself is threadsafe, but we don't want our receives and transmits
	  * to get mixed up.
	  */
	RTT::os::Mutex              eCatLock;
	
	/** @brief Sleeps for a certain number of nanoseconds.
	  * @param sleeptime The time to sleep for.
	  */
	void nanosleep(RTT::os::TimeService::nsecs sleeptime);
	
	public:
		/** @brief Initializes the EtherCAT master
		  * @param rt_ops A pointer to the RT Operations component main class
		  */
		ECatComm(RTOps* rt_ops);
		
		/** @brief Actually inits the ECat master. Return true on success.
		  */
		bool initialize();
		
		/** @brief  Checks for slaves.
		  * @return Whether or not it was successful.
		  */
		bool init();
		
		/** @brief Executes the main loop.
		  */
		void loop();
		
		/** @brief  Stops the main loop.
		  */
		bool breakLoop();
		
		/** @brief  Shuts down medullas and SOEM.
		  * @return Whether it was successful.
		  */
		void finalize();
		
		/** @brief Processes and sends new torque values.
		  */
		void transmitHook();
		
		/** @brief Disable the robot */
		void disable();
		
		/** @brief Enable the robot */
		void enable();
		
		/** @brief Command an ESTOP.
		  */
		void eStop();
		
		/** @brief Leave ESTOP mode */
		void leaveEStop();
};

#endif // ECATCOMM_H
