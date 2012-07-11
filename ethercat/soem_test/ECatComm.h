#ifndef ECATCOMM_H
#define ECATCOMM_H

/** @file
  * @brief Interfaces between the Orocos stuff and the EtherCAT Master.
  */

#include "TestMedulla.h"
#include "RobotComm.h"
extern "C" {
#include <ethercatmain.h>
#include <ethercattype.h>
#include <ethercatconfig.h>
#include <ethercatdc.h>
}
#include <stdio.h>
#include <stdlib.h>

#define VENDOR_ID        0x60f

#define EC_TIMEOUT_US      500
#define LOOP_PERIOD_NS 1000000

//! @brief Performs the actual interaction with the EtherCAT Master
class ECatComm {
	//! @brief Holds the callback for when we've received data
	void       (*recv_callback)();
	
	//! @brief A pointer to the object holding all the process data
	RobotComm*               data;
	
	// What does this do? It's in simple_test... but the Doxygen docs don't really help :(
	char IOmap[4096];
	
	/** @brief Holds whether or not the master's been initialized... so cyclicTask() and transmit()
	  * don't fail horribly.
	  */
	bool initialized;
	
	public:
		/** This initializes the EtherCAT master */
		ECatComm();
		
		/** @brief Actually brings up and initializes the ethercat slaves --
		  * blocks until the slaves hit OP mode.
		  */
		void init();
		
		//! @brief Sends out the torque-setting frame to the Medullas
		void transmitHook();
		
		//! @brief This should be called periodically by the ECatTimer
		void cyclicTask();
		
		/** @brief This allows the Orocos-interface code to access the data
		  * going to and from the medullas
		  * @return A pointer to the robotComm class containing the Medulla data.
		  */
		RobotComm* getData();
		
		/** @brief Registers a callback to be executed after data is received.
		  * @param dataCallback A pointer to the callback function
		  */
		void registerDataCallback(void (*dataCallback)());
};

#endif // ECATCOMM_H
