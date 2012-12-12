#ifndef CONTROLLERCOMMON_HPP
#define CONTROLLERCOMMON_HPP

/** @file controller_common.hpp
  * @brief Contains structures used by both the GUI and the controller itself.
  */

namespace atrias {
namespace controller {

/** @brief The state for the main state machine for this controller.
  */
typedef int8_t State_t;
enum class State: State_t {
	INIT = 0, // For smooth initialization
	FLIGHT,   // While we're in-flight
	STANCE,   // For during stance
	LOCKED    // To let us stop bouncing w/out breaking
};

}

}

#endif //CONTROLLERCOMMON_HPP

// vim: noexpandtab
