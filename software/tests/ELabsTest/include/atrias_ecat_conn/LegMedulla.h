#ifndef LEGMEDULLA_H
#define LEGMEDULLA_H

/** @file
  * @brief Provides code specific to the Leg Medulla
  */

// Orocos
#include <rtt/os/TimeService.hpp>

#include <stdint.h>

#include <atrias_msgs/controller_output.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_shared/globals.h>
#include "robot_invariant_defs.h"
#include "robot_variant_defs.h"
#include "atrias_ecat_conn/Medulla.h"

namespace atrias {

namespace ecatConn {

/** @brief Contains pointers to the data received from and transmitted to this
  * type of Medulla.
  */
class LegMedulla : public Medulla {
	// Stuff sent to the medulla
	uint8_t*        command;
	uint16_t*       counter;
	int32_t*        motorCurrent;
	
	// Stuff received from the Medulla
	uint8_t*        id;
	uint8_t*        state;
	uint8_t*        timingCounter;
	uint8_t*        errorFlags;
	uint8_t*        limitSwitch;
	uint16_t*       toeSensor;
	
	uint32_t*       motorEncoder;
	int16_t*        motorEncoderTimestamp;
	
	uint16_t*       incrementalEncoder;
	uint16_t*       incrementalEncoderTimestamp;
	
	uint32_t*       legEncoder;
	int16_t*        legEncoderTimestamp;
	
	uint16_t*       motorVoltage;
	uint16_t*       logicVoltage;
	
	uint16_t*       thermistor0;
	uint16_t*       thermistor1;
	uint16_t*       thermistor2;
	uint16_t*       thermistor3;
	uint16_t*       thermistor4;
	uint16_t*       thermistor5;
	
	int16_t*        amp1MeasuredCurrent;
	int16_t*        amp2MeasuredCurrent;
	
	// Used for processing
	int64_t         motorEncoderValue;
	int64_t         legEncoderValue;
	int16_t         motorEncoderTimestampValue;
	int16_t         legEncoderTimestampValue;
	uint16_t        incrementalEncoderValue;
	uint16_t        incrementalEncoderTimestampValue;
	uint8_t         timingCounterValue;
	
	/** @brief Calculates the current command to send to the Medulla.
	  * @return The value that should be sent to this Medulla.
	  */
	int32_t      calcMotorCurrentOut(atrias_msgs::controller_output& controllerOutput);
	
	/** @brief  Converts encoder ticks to radians.
	  * @param  ticks The encoder's reported position.
	  * @param  calib_val The position, in ticks, at which this encoder was calibrated
	  * @param  rad_per_tick The radians per tick setting for this encoder
	  * @param  calib_val_rad The position in radians at which this encoder was calibrated
	  * @return This encoder's position in radians.
	  */
	double       encTicksToRad(uint32_t ticks, uint32_t calib_val, double rad_per_tick, double calib_val_rad);

	/** @brief Decodes and stores the new positions for the legs and motors.
	  */
	void         processPositions(atrias_msgs::robot_state& robotState);

	/** @brief Processes and stores the new velocities for the legs and motors.
	  */
	void         processVelocities(RTT::os::TimeService::nsecs deltaTime, atrias_msgs::robot_state& robotState);
	
	/** @brief Process all the thermistor values.
	  */
	void         processThermistors(atrias_msgs::robot_state& robotState);
	
	/** @brief Reads in all the limit switches and updates robotState.
	  */
	void         processLimitSwitches();
	
	/** @brief Processes the motor and logic voltages.
	  */
	void         processVoltages(atrias_msgs::robot_state& robotState);
	
	/** @brief Processes the motor currents and updates the motor states.
	  */
	void         processCurrents(atrias_msgs::robot_state& robotState);
	
	/** @brief Does the processing for the motor's internal incremental encoders.
	  * @param deltaTime The time between the DC clock signals for the last and this cycle.
	  */
	void         processIncrementalEncoders(RTT::os::TimeService::nsecs deltaTime, atrias_msgs::robot_state& robotState);
	
	public:
		/** @brief Does SOEM's slave-specific init.
		  * @param inputs  A pointer to this slave's inputs
		  * @param outputs A pointer to this slave's outputs;
		  */
		LegMedulla(uint8_t* inputs, uint8_t* outputs);
		
		/** @brief Returns the total inputs size for this medulla type.
		  * @return The total inputs size for this medulla type.
		  */
		intptr_t getInputsSize();
		
		/** @brief Returns the total outputs size for this medulla type.
		  * @return The total outputs size for this medulla type.
		  */
		intptr_t getOutputsSize();
		
		/** @brief Tells this medulla to read in data for transmission.
		  */
		void processTransmitData(atrias_msgs::controller_output& controller_output);
		
		/** @brief Tells this Medulla to update the robot state.
		  */
		void processReceiveData(atrias_msgs::robot_state& robot_state);
		
		/** @brief Gets the ID of this medulla
		  * @return The medulla's ID.
		  */
		uint8_t getID();
};

}

}

#endif // LEGMEDULLA_H
