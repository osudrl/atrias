#ifndef IMUMEDULLA_H
#define IMUMEDULLA_H

// Orocos
#include <rtt/os/TimeService.hpp>

#include <rtt/Logger.hpp>

#include <stdint.h>
#include <math.h>

#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <robot_invariant_defs.h>
#include <robot_variant_defs.h>
#include <atrias_shared/globals.h>
#include "atrias_medulla_drivers/Medulla.h"

namespace atrias {
namespace medullaDrivers {

class ImuMedulla : public Medulla {
	// Stuff sent to Medulla (i.e., RxPDO entries)
	uint8_t  *command;   // This is a medulla_state_t elsewhere.
	uint16_t *counter;

	// Stuff received from Medulla (i.e., TxPDO entries)
	uint8_t  *id;
	uint8_t  *state;   // This is a medulla_state_t elsewhere.
	uint8_t  *timingCounter;
	uint8_t  *errorFlags;
	float    *gyrX;   // IEEE-754 SPFP
	float    *gyrY;   // IEEE-754 SPFP
	float    *gyrZ;   // IEEE-754 SPFP
	float    *accX;   // IEEE-754 SPFP
	float    *accY;   // IEEE-754 SPFP
	float    *accZ;   // IEEE-754 SPFP
	uint8_t  *status;   // IMU status, 0x77 if all good.
	uint8_t  *seq;   // Increments from 0-127.
	int16_t  *temperature;   // In deg C, nominally around 42 deg max.
	uint32_t *crc;   // 32-bit CRC

	// The following variables are used for processing
	uint8_t timingCounterValue;

	/** @brief Integrated pitch, drift and all. In radians.
	  */
	double pitch;

	/** @brief The PDOEntryDatas array.
	  */
	PDOEntryData pdoEntryDatas[MEDULLA_IMU_TX_PDO_COUNT+MEDULLA_IMU_RX_PDO_COUNT];

	/**
	 * @brief Decodes and stores the new values from the IMU.
	 *
	 * Currently, this only integrates the Z delta angle and treats it like the
	 * boom pitch encoder.
	 *
	 * @param deltaTime The time between this DC cycle and the last DC cycle.
	 * @param robotState The robot state in which to store the new values.
	 */
	void processIMU(RTT::os::TimeService::nsecs deltaTime,
			atrias_msgs::robot_state &robotState);

	public:
		/** @brief Does the slave-specific init.
		  */
		ImuMedulla();

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

} // medullaDrivers
} // atrias

#endif // IMUMEDULLA_H
