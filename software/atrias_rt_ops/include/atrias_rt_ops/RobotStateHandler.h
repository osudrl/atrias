#ifndef ROBOTSTATEHANDLER_H
#define ROBOTSTATEHANDLER_H

/** @file
  * @brief This contains a class protecting access to and from the robot state.
  */

class RobotStateHandler;

// Orocos
#include <rtt/os/Mutex.hpp>
#include <rtt/os/MutexLock.hpp>

#include <atrias_msgs/robot_state.h>
#include <robot_invariant_defs.h>

#include "atrias_rt_ops/RTOps.h"

namespace atrias {

namespace rtOps {

class RobotStateHandler {
	/** @brief Holds a pointer to the main RTOps class.
	  */
	RTOps*                   rtOps;
	
	/** @brief Holds the current robot state.
	  */
	atrias_msgs::robot_state robotState;
	
	/** @brief Protects access to \a robotState. For thread safety.
	  */
	RTT::os::Mutex           robotStateLock;
	
	public:
		/** @brief Initializes the RobotStateHandler.
		  * @param rt_ops A pointer to the main RT Ops class.
		  */
		RobotStateHandler(RTOps* rt_ops);
		
		/** @brief Returns the current robot state (threadsafe).
		  * @return The current robot state.
		  */
		atrias_msgs::robot_state getRobotState();
		
		/** @brief Sets the robot state (threadsafe).
		  * @param newState The new robot state.
		  */
		void setRobotState(atrias_msgs::robot_state &newState);
};

}

}

#endif // ROBOTSTATEHANDLER_H
