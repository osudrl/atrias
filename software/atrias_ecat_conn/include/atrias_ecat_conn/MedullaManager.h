#ifndef MEDULLAMANAGER_H
#define MEDULLAMANAGER_H

/** @file
  * Handles all of our Medulla objects.
  */

#include <atrias_msgs/robot_state.h>

namespace atrias {

namespace ecatConn {

class MedullaManager {
	public:
		/** @brief Processes our receive data into the robot state.
		  */
		void processReceiveData();
		
		/** @brief Allows access to the robot state.
		  * @return The robot state.
		  */
		atrias_msgs::robot_state getRobotState();
};

}

}

#endif // MEDULLAMANAGER_H
