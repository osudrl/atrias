#include "atrias_rt_ops/StopController.h"

namespace atrias {
namespace rtOps {

void StopController::disabled() {

}

atrias_msgs::controller_output StopController::runController(atrias_msgs::robot_state &robotState) {
	atrias_msgs::controller_output co;
	return co;
}

}
}

// vim: noexpandtab
