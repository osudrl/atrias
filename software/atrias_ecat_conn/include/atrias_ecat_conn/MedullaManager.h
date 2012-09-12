#ifndef MEDULLAMANAGER_H
#define MEDULLAMANAGER_H

/** @file
  * Handles all of our Medulla objects.
  */

// Orocos
#include <rtt/os/TimeService.hpp>
#include <rtt/Logger.hpp>

// SOEM
extern "C" {
#include <ethercattype.h>
#include <ethercatmain.h>
}

#include <stdint.h>

#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/globals.h>
#include <atrias_medulla_drivers/BoomMedulla.h>
#include <atrias_medulla_drivers/LegMedulla.h>
#include <atrias_medulla_drivers/HipMedulla.h>
#include <atrias_medulla_drivers/Medulla.h>

namespace atrias {

namespace ecatConn {

class MedullaManager {
	// All of our Medullas:
	medullaDrivers::LegMedulla*  lLegA;
	medullaDrivers::LegMedulla*  lLegB;
	medullaDrivers::LegMedulla*  rLegA;
	medullaDrivers::LegMedulla*  rLegB;
	medullaDrivers::BoomMedulla* boom;
	medullaDrivers::HipMedulla*  lLegHip;
	medullaDrivers::HipMedulla*  rLegHip;
	
	/** @brief Holds our robot state for us.
	  * Note: Functions using this are NOT thread-safe and should only be called
	  *       ECat receive thread.
	  */
	atrias_msgs::robot_state robotState;
	
	/** @brief Does the slave card-specific init.
	  */
	void slaveCardInit(ec_slavet slave);
	
	/** @brief Does init specific to operation w/ actual medullas.
	  */
	void medullasInit(ec_slavet slaves[], int slavecount);
	
	/** @brief Initialize the boom medulla.
	  * @param slave The ECat slave for this Medulla.
	  */
	void initBoomMedulla(ec_slavet slave);
		
	/** @brief Fills in a PDORegData struct.
	  * @param pdo_reg_data The PDORegData struct to be filled in.
	  * @param inputs A pointer to the slave's inputs.
	  * @param outputs A pointer to the slave's outputs.
	  */
	void fillInPDORegData(medullaDrivers::PDORegData pdo_reg_data, uint8_t* outputs, uint8_t* inputs);
	
	/** @brief Identifies the robot's configuration from the created Medullas.
	  * @return The robot's configuration.
	  */
	rtOps::RobotConfiguration calcRobotConfiguration();
	
	/** @brief Configures the array of pointers for a medulla's inputs[] parameter.
	  * @param slave_inputs The address to this slave's inputs.
	  * @param inputs_array The array in which to store the values.
	  * @param num_entries  The number of elements in the inputs_array param.
	  */
	void InputsConfig(uint8_t* slave_inputs, intptr_t* inputs_array, int num_entries);
	
	/** @brief Configures the array of pointers for a medulla's outputs[] parameter.
	  * @param slave_outputs The address to this slave's outputs.
	  * @param outputs_array The array in which to store the values.
	  * @param num_entries  The number of elements in the inputs_array param.
	  */
	void OutputsConfig(uint8_t* slave_outputs, intptr_t* outputs_array, int num_entries);
	
	public:
		/** @brief Initializes the MedullaManager.
		  */
		MedullaManager();
		
		/** @brief Cleans up the MedullaManager.
		  */
		~MedullaManager();
		
		/** @brief Inits the medullas
		  */
		void start(ec_slavet slaves[], int slavecount);
		
		/** @brief Processes our receive data into the robot state.
		  */
		void processReceiveData();
		
		/** @brief Processes controller outputs into SOEM's buffer.
		  * @param controller_output The controller output.
		  */
		void processTransmitData(atrias_msgs::controller_output& controller_output);
		
		/** @brief Sets the timestamp in robot state.
		  * @param time The current time in nanoseconds.
		  */
		void setTime(RTT::os::TimeService::nsecs time);
		
		/** @brief Allows access to the robot state.
		  * @return The robot state.
		  */
		atrias_msgs::robot_state getRobotState();
		
		/** @brief Lets other classes control the robotConfiguration.
		  * @param new_robot_configuration The new robot configuration.
		  */
		void setRobotConfiguration(rtOps::RobotConfiguration new_robot_configuration);
};

}

}

#endif // MEDULLAMANAGER_H
