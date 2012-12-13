#ifndef CONTROLLERCOMMON_HPP
#define CONTROLLERCOMMON_HPP

/** @file controller_common.hpp
  * @brief Contains structures used by both the GUI and the controller itself.
  */

#include <cstdint>

#include <atrias_shared/globals.h>

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

/** @brief This is the metadata for this controller's custom events
  */
enum class EventMetadata: rtOps::RtOpsEventMetadata_t {
	BAD_MAIN_STATE = 1
};

}

}

#endif //CONTROLLERCOMMON_HPP

// vim: noexpandtab
