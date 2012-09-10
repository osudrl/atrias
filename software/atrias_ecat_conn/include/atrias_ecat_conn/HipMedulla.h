#ifndef HIPMEDULLA_H
#define HIPMEDULLA_H

// Orocos
#include <rtt/os/TimeService.hpp>

#include <stdint.h>

#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/robot_state_hip.h>
#include <atrias_msgs/controller_output.h>
#include <robot_invariant_defs.h>
#include <robot_variant_defs.h>
#include <atrias_shared/globals.h>
#include "atrias_ecat_conn/Medulla.h"

namespace atrias {

namespace ecatConn {

class HipMedulla : public Medulla {
	// Stuff sent to the medulla
	uint8_t*  command;
	uint16_t* counter;
	int32_t*  motorCurrent;
	
	// Stuff received from the medulla
	uint8_t*  id;
	uint8_t*  state;
	uint8_t*  timingCounter;
	uint8_t*  errorFlags;
	uint8_t*  limitSwitches;
	// The rest aren't implemented yet due to impending changes.
	
	
	uint8_t   timingCounterValue;
	
	/** @brief Calculate the current command to send to this Medulla.
	  * @param controllerOutput The controller output from which to pull
	  * the current command.
	  * @return The motor current command value.
	  */
	int32_t calcMotorCurrentOut(atrias_msgs::controller_output& controllerOutput);
	
	public:
		/** @brief Does SOEM's slave-specific init.
		  * @param inputs A pointer to this slave's inputs.
		  * @param outputs A pointer to this slave's outputs.
		  */
		HipMedulla(uint8_t* inputs, uint8_t* outputs);
		
		/** @brief Gets this medulla's ID.
		  * @return This medulla's ID.
		  */
		uint8_t getID();
		
		/** @brief Tells this medulla to read in data for transmission.
		  */
		void processTransmitData(atrias_msgs::controller_output& controller_output);
		
		/** @brief Tells this Medulla to update the robot state.
		  */
		void processReceiveData(atrias_msgs::robot_state& robot_state);
};

}

}

#endif // HIPMEDULLA_H
