#ifndef STOPCONTROLLER_H
#define STOPCONTROLLER_H

/** @file StopController.h
  * @brief This contains the shutdown (hip relaxation) controller.
  */

#include <atrias_msgs/controller_output.h>
#include <atrias_msgs/robot_state.h>

namespace atrias {

namespace rtOps {

class StopController {
	public:
		/** @brief This should be run when this controller is disabled.
		  */
		void disabled();

		/** @brief This is equivalent to a ATC's runController operation.
		  * @param robotState The current robot state.
		  * @return Controller output
		  */
		atrias_msgs::controller_output runController(atrias_msgs::robot_state &robotState);
};

}

}

#endif // STOPCONTROLLER_H

// vim: noexpandtab
