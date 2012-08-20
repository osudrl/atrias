#ifndef MEDULLAMANAGER_H
#define MEDULLAMANAGER_H

/** @file
  * @brief Contains a class that manages the medullas for the entire robot.
  */

extern "C" {
#include <ethercattype.h>
#include <ethercatmain.h>
#include <ethercatdc.h>
}

// Orocos
#include <rtt/Logger.hpp>

#include "robot_invariant_defs.h"
#include "atrias_rt_ops/RTOps.h"
#include "atrias_rt_ops/ECatSlavesManager.h"
#include "atrias_rt_ops/LegMedulla.h"

class MedullaManager : public ECatSlavesManager {
	/** @brief Holds a pointer to the RT OPS component for access to
	  * the controller output and robot state.
	  */
	RTOps*          rtOps;
	
	/* Medullas! */
	LegMedulla*     LlegA;
	LegMedulla*     LlegB;
	LegMedulla*     RlegA;
	LegMedulla*     RlegB;
	
	/** @brief Holds whether the gui has commanded an enable.
	  */
	bool            cmd_enabled;
	
	/** @brief Used by \a leaveEStop() to signal \a updateStateCmd()
	  * to command the medullas to reset state.
	  */
	bool            leave_estop;
	
	/** @brief Holds the desired robot state.
	  */
	medulla_state_t state_cmd;
	
	/** @brief Checks whether all medullas are estopped.
	  * @return Whether all medullas are estopped.
	  */
	bool getAllEStopped();
	
	/** @brief Computes the state command for the medullas.
	  */
	void updateStateCmd();
	
	/** @brief Contains the safety code.
	  * @return Whether or not the medullas should be commanded into halt mode.
	  */
	bool shouldHalt();
	
	public:
		/** Initializes the MedullaManager */
		MedullaManager(RTOps* rt_ops, ec_slavet slaves[], int slavecount);
		
		/** @brief Processes the data from the Medullas */
		void processReceiveData();
		
		/** @brief Process and prepare the new controller output data */
		void processTransmitData();
		
		/** @brief Enable or disable the robot.
		  * @param enabled Whether the bot should be enabled or not.
		  */
		void setEnabled(bool enabled);
		
		/** @brief Command all the Medullas to ESTOP */
		void eStop();
		
		/** @brief Take the Medullas out of EStop
		  */
		void leaveEStop();
};

#endif // MEDULLAMANAGER_H
