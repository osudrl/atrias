#include "atrias_rt_ops/RobotStateHandler.h"

namespace atrias {

namespace rtOps {

RobotStateHandler::RobotStateHandler(RTOps* rt_ops) {
	rtOps = rt_ops;
}

atrias_msgs::robot_state RobotStateHandler::getRobotState() {
	RTT::os::MutexLock lock(robotStateLock);
	robotState.rtOpsState = (RtOpsState_t) rtOps->getStateMachine()->getState();
	return robotState;
}

void RobotStateHandler::setRobotState(atrias_msgs::robot_state &newState) {
	RTT::os::MutexLock lock(robotStateLock);
	robotState = newState;
}

}

}

// vim: noexpandtab
