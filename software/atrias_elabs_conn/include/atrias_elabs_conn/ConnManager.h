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

#include <time.h>

#include <atrias_shared/globals.h>
#include <atrias_msgs/controller_output.h>
#include <robot_invariant_defs.h>
#include "atrias_elabs_conn/ELabsConn.h"
#include "atrias_elabs_conn/cstructs.h"

#define RECEIVE_WAIT_TIME_NS 300000

namespace atrias {

namespace elabsConn {

class ConnManager : RTT::Activity {
	/** @brief Lets us access what we need to in ELabsConn.
	  */
	ELabsConn*     eLabsConn;
	
	/** @brief Protects access to EtherLabs's functions and data.
	  */
	RTT::os::Mutex eCatLock;
	
	/** @brief Handles all our medulla objects.
	  */
	MedullaManager* medullaManager;
	
	/** @brief Does the cyclic EtherCAT stuff. Called by loop and init (while waiting for OP).
	  */
	void cyclic();
	
	public:
		/** @brief The constructor.
		  * @param elabs_conn A pointer to the ELabsConn instance.
		  */
		ConnManager(ELabsConn* elabs_conn);
		
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
		
		/** @brief Receives and processes data. Called cyclicly.
		  */
		void loop();
		
		/** @brief Sends new outputs over ECat.
		  * @param controller_output The new outputs.
		  */
		void sendControllerOutput(atrias_msgs::controller_output& controller_output);
};

}

}

#endif // CONNMANAGER_H
