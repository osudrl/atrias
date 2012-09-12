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
#include "atrias_medulla_drivers/Medulla.h"

namespace atrias {

namespace medullaDrivers {

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
	
	uint32_t* hipEncoder;
	uint16_t* hipEncoderTimestamp;
	
	uint16_t* motorVoltage;
	uint16_t* logicVoltage;
	
	uint16_t* thermistor0;
	uint16_t* thermistor1;
	uint16_t* thermistor2;
	
	int16_t*  ampMeasuredCurrent;
	
	float*    accelX;
	float*    accelY;
	float*    accelZ;
	float*    angRateX;
	float*    angRateY;
	float*    angRateZ;
	float*    m11;
	float*    m12;
	float*    m13;
	float*    m21;
	float*    m22;
	float*    m23;
	float*    m31;
	float*    m32;
	float*    m33;
	int32_t*  timer;
	
	uint16_t* incrementalEncoder;
	uint16_t* incrementalEncoderTimestamp;
	
	
	uint8_t   timingCounterValue;
	uint16_t  incrementalEncoderValue;
	int16_t   incrementalEncoderTimestampValue;
	bool      incrementalEncoderInitialized;
	
	/** @brief Calculate the current command to send to this Medulla.
	  * @param controllerOutput The controller output from which to pull
	  * the current command.
	  * @return The motor current command value.
	  */
	int32_t calcMotorCurrentOut(atrias_msgs::controller_output& controllerOutput);
	
	/** @brief Updates the limit switch values in robotState w/ the
	  * new values from the Medulla.
	  * @param hip The robot_state_hip in which to store the new values.
	  */
	void    updateLimitSwitches(atrias_msgs::robot_state_hip& hip);
	
	/** @brief Updates the position and velocities from the encoders.
	  * @param delta_time The delta time, in nsecs, between the relevant DC
	  *        clock cycles.
	  * @param hip The robot_state_hip in which to store the new values.
	  */
	void    updateEncoderValues(RTT::os::TimeService::nsecs delta_time,
	                            atrias_msgs::robot_state_hip& hip);
	
	/** @brief The PDOEntryDatas array.
	  */
	PDOEntryData pdoEntryDatas[MEDULLA_HIP_TX_PDO_COUNT+MEDULLA_HIP_RX_PDO_COUNT];
	
	public:
		/** @brief Does the slave-specific init.
		  */
		HipMedulla();
		
		/** @brief Returns a \a PDORegData struct for PDO entry location.
		  * @return A PDORegData struct w/ sizes filled out.
		  */
		PDORegData getPDORegData();
		
		/** @brief Does all post-Ops init.
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

#endif // HIPMEDULLA_H
