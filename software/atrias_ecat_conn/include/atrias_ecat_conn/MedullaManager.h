#ifndef MEDULLAMANAGER_H
#define MEDULLAMANAGER_H

/** @file
  * Handles all of our Medulla objects.
  */

// Orocos
#include <rtt/os/TimeService.hpp>

#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>

namespace atrias {

namespace ecatConn {

class MedullaManager {
	public:
		/** @brief Processes our receive data into the robot state.
		  */
		void processReceiveData();
		
		/** @brief Processes controller outputs into SOEM's buffer.
		  * @param controller_output The controller output.
		  */
		void processTransmitData(atrias_msgs::controller_output);
		
		/** @brief Sets the timestamp in robot state.
		  * @param time The current time in nanoseconds.
		  */
		void setTime(RTT::os::TimeService::nsecs time);
		
		/** @brief Allows access to the robot state.
		  * @return The robot state.
		  */
		atrias_msgs::robot_state getRobotState();
};

}

}

#endif // MEDULLAMANAGER_H
