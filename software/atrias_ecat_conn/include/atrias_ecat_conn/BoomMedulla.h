#ifndef BOOMMEDULLA_H
#define BOOMMEDULLA_H

// Orocos
#include <rtt/os/TimeService.hpp>

#include <stdint.h>

#include <atrias_msgs/robot_state.h>
#include <robot_invariant_defs.h>
#include <robot_variant_defs.h>
#include "atrias_ecat_conn/Medulla.h"

namespace atrias {

namespace ecatConn {

class BoomMedulla : public Medulla {
	// Stuff sent to the medulla
	uint8_t*  command;
	uint16_t* counter;
	
	// Stuff received from the medulla
	uint8_t*  id;
	uint8_t*  state;
	uint8_t*  timingCounter;
	uint8_t*  errorFlags;
	
	uint32_t* xEncoder;
	uint16_t* xTimestamp;
	
	uint32_t* pitchEncoder;
	uint16_t* pitchTimestamp;
	
	uint32_t* zEncoder;
	uint16_t* zTimestamp;
	
	uint16_t* logicVoltage;
	
	// The following variables are used for processing
	uint8_t   timingCounterValue;
	
	/** @brief The last value of the pitch encoder. Used to find position deltas.
	  */
	uint32_t  pitchEncoderValue;
	
	/** @brief The absolute pitch encoder position, in encoder ticks.
	  * After initialization, 0 should corrospond to vertical.
	  */
	int32_t   pitchEncoderPos;
	
	/** @brief Decodes and stores the new value from the pitch encoder.
	  * @param deltaTime The time between this DC cycle and the last DC cycle.
	  * @param robotState The robot state in which to store the new values.
	  */
	void      processPitchEncoder(RTT::os::TimeService::nsecs deltaTime,
	                              atrias_msgs::robot_state&   robotState);
	
	/** @brief Decodes and stores the new value from the height encoder.
	  * @param deltaTime The time between this DC cycle and the last DC cycle.
	  * @param robotState The robot state in which to store the new values.
	  */
	void      processZEncoder(RTT::os::TimeService::nsecs deltaTime,
	                          atrias_msgs::robot_state&   robotState);
	
	public:
		/** @brief Does SOEM's slave-specific init.
		  * @param inputs A pointer to this slave's inputs.
		  * @param outputs A pointer to this slave's outputs.
		  */
		BoomMedulla(uint8_t* inputs, uint8_t* outputs);
		
		/** @brief Tells this Medulla to update the robot state.
		  */
		void processReceiveData(atrias_msgs::robot_state& robot_state);
};

}

}

#endif // BOOMMEDULLA_H
