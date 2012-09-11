#ifndef CONNMANAGER_H
#define CONNMANAGER_H

/** @file
  * Contains a class which runs does the main EtherCAT communications in the 
  * ECat Connector.
  */

class ConnManager;

// Orocos
#include <rtt/Activity.hpp>
#include <rtt/os/Mutex.hpp>
#include <rtt/os/MutexLock.hpp>
#include <rtt/Logger.hpp>
#include <rtt/os/TimeService.hpp>

// SOEM
extern "C" {
#include <ethercattype.h>
#include <ethercatmain.h>
#include <ethercatconfig.h>
#include <ethercatdc.h>
}

#include <signal.h>

#include "atrias_ecat_conn/ECatConn.h"
#include <atrias_msgs/controller_output.h>
#include <robot_invariant_defs.h>
#include <atrias_shared/globals.h>

#define EC_TIMEOUT_US      500
#define TIMING_FILTER_GAIN 100

namespace atrias {

namespace ecatConn {

class ConnManager : public RTT::Activity {
	/** @brief Lets us access what we need to in ECatConn.
	  */
	ECatConn*      eCatConn;
	
	/** @brief This is where SOEM stores its data.
	  */
	char           IOmap[4096];
	
	/** @brief Used by \a breakLoop() to stop the main loop.
	  */
	bool           done;
	
	/** @brief Protects access to SOEM's data.
	  */
	RTT::os::Mutex eCatLock;
	
	/** @brief Sends and receives an EtherCAT frame.
	  * Does not grab eCatLock -- must already have it.
	  */
	void           cycleECat();
	
	/** @brief Used to keep the loop's phase offset constant from cycle
	  * to cycle and to compensate for overshoots.
	  */
	RTT::os::TimeService::nsecs targetTime;
	
	/** @brief Used to detect missed deadlines.
	  */
	bool           midCycle;
	
	public:
		/** @brief The constructor.
		  * @param ecat_conn A pointer to the ECatConn instance.
		  */
		ConnManager(ECatConn* ecat_conn);
		
		/** @brief Shuts down EtherCAT.
		  */
		~ConnManager();
		
		/** @brief Does most of the basic EtherCAT configuration.
		  * @return Success
		  */
		bool configure();
		
		/** @brief Start the main loop.
		  * @return Success.
		  */
		bool initialize();
		
		/** @brief The main ECat receive loop.
		  */
		void loop();
		
		/** @brief Sends new outputs over ECat.
		  * @param controller_output The new outputs.
		  */
		void sendControllerOutput(atrias_msgs::controller_output& controller_output);
		
		/** @brief Stops the main loop.
		  * @return Success
		  */
		bool breakLoop();
};

}

}

#endif // CONNMANAGER_H
