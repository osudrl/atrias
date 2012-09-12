#ifndef BOOMMEDULLA_H
#define BOOMMEDULLA_H

// Orocos
#include <rtt/os/TimeService.hpp>

#include <rtt/Logger.hpp>

#include <stdint.h>

#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <robot_invariant_defs.h>
#include <robot_variant_defs.h>
#include <atrias_shared/globals.h>
#include "atrias_medulla_drivers/Medulla.h"

namespace atrias {

namespace medullaDrivers {

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
	
	/** @brief The last value of the X encoder. Used to find position deltas.
	  */
	uint32_t  xEncoderValue;
	
	/** @brief The last value of *xTimestamp.
	  */
	int16_t   xTimestampValue;
	
	/** @brief The last value of the pitch encoder. Used to find position deltas.
	  */
	uint32_t  pitchEncoderValue;
	
	/** @brief The absolute pitch encoder position, in encoder ticks.
	  * After initialization, 0 should correspond to vertical.
	  */
	int32_t   pitchEncoderPos;
	
	/** @brief Stores the last value of *pitchTimestamp
	  * Used for timestamp delta calculation.
	  */
	int16_t   pitchTimestampValue;
	
	/** @brief The last value of the Z encoder, for position delta calculation.
	  */
	uint32_t  zEncoderValue;
	
	/** @brief The absolute z encoder position, in encoder ticks.
	  * After initialization, 0 should correspond to BOOM_Z_CALIB_LOC.
	  */
	int32_t   zEncoderPos;
	
	/** @brief Stores the last value of *zTimestamp
	  */
	int16_t   zTimestampValue;
	
	/** @brief The PDOEntryDatas array.
	  */
	PDOEntryData pdoEntryDatas[MEDULLA_BOOM_TX_PDO_COUNT+MEDULLA_BOOM_RX_PDO_COUNT];
	
	/** @brief Decodes and stores the new values from the X encoder.
	  * @param deltaTime The time between this DC cycle and the last DC cycle.
	  * @param robotState The robot state in which to store the new values.
	  */
	void      processXEncoder(RTT::os::TimeService::nsecs deltaTime,
	                          atrias_msgs::robot_state&   robotState);
	
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
		/** @brief Does the slave-specific init.
		  */
		BoomMedulla();
		
		/** @brief Returns a \a PDORegData struct for PDO entry location.
		  * @return A PDORegData struct w/ sizes filled out.
		  */
		PDORegData getPDORegData();
		
		/** @brief Does all post-Ops init.
		  * @param pdo_reg_data The filled-out PDORegData struct.
		  */
		void postOpInit();
		
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

#endif // BOOMMEDULLA_H
