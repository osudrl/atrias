#ifndef LEGMEDULLA_H
#define LEGMEDULLA_H

/** @file
  * @brief Provides code specific to the Leg Medulla
  */

// Orocos
#include <rtt/os/TimeService.hpp>
#include <rtt/Logger.hpp>

#include <stdint.h>

#include <atrias_msgs/controller_output.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_shared/globals.h>
#include "robot_invariant_defs.h"
#include "robot_variant_defs.h"
#include "atrias_medulla_drivers/Medulla.h"

namespace atrias {

namespace medullaDrivers {

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
	
	uint32_t*       legEncoder;
	int16_t*        legEncoderTimestamp;
	
	uint16_t*       incrementalEncoder;
	uint16_t*       incrementalEncoderTimestamp;

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

	uint16_t*       kneeForce1;
	uint16_t*       kneeForce2;
	
	// Used for processing
	int64_t         motorEncoderValue;
	int64_t         legEncoderValue;
	int32_t         motorEncoderTimestampValue;
	int32_t         legEncoderTimestampValue;
	double          incrementalEncoderStart;
	long            incrementalEncoderPos;
	uint16_t        incrementalEncoderValue;
	uint16_t        incrementalEncoderTimestampValue;
	uint8_t         timingCounterValue;
	double          legPositionOffset;
	int16_t         zeroToeSensor;
	bool            newToeBool;
	bool            oldToeBool;
	bool            toeBool;
	uint16_t        toeCounter;
	
	// Whether or not the encoder value for this cycle was erroneous
	bool            skipMotorEncoder;
	bool            skipLegEncoder;
	
	/** @brief The PDOEntryDatas array.
	  */
	PDOEntryData pdoEntryDatas[MEDULLA_LEG_TX_PDO_COUNT+MEDULLA_LEG_RX_PDO_COUNT];
	
	/** @brief Check for spikes in the encoder data.
	  */
	void         checkErroneousEncoderValues();
	
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
	  * @param robotState The robot_state in which to store the new values.
	  * @param reset Whether or not to reset the limit switch values.
	  */
	void         processLimitSwitches(atrias_msgs::robot_state& robotState, bool reset);

	/** @brief Reads in and processes the strain gauge values.
	  * @param robotState The robot_state in which to store the new values.
	  */
	void         processStrainGauges(atrias_msgs::robot_state&  robotState);
	
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

	/** @brief Detects whether the toe is on the ground, based on its force reading.
	  * @return True if it is, false if it isn't
	  */
	bool         toeDetect();
	
	/** @brief Updates the value of legPositionOffset and incrementalEncoderStart.
	  */
	void         updatePositionOffsets();
	
	public:
		/** @brief Does the slave-specific init.
		  */
		LegMedulla();
		
		/** @brief Returns a \a PDORegData struct for PDO entry location.
		  * @return A PDORegData struct w/ sizes filled out.
		  */
		PDORegData getPDORegData();
		
		/** @brief Does all post-Op init.
		  */
		void postOpInit();
		
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
