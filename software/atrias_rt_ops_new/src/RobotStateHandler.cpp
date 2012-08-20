#include "atrias_rt_ops/RobotStateHandler.h"

namespace atrias {

namespace rtOps {

RobotStateHandler::RobotStateHandler(RTOps* rt_ops) {
	rtOps = rt_ops;
}

atrias_msgs::robot_state RobotStateHandler::getRobotState() {
	RTT::os::MutexLock lock(robotStateLock);
	return robotState;
}

void RobotStateHandler::setRobotState(atrias_msgs::robot_state &newState) {
	checkForNewErrors(newState);
	
	RTT::os::MutexLock lock(robotStateLock);
	robotState = newState;
}

void RobotStateHandler::checkForNewErrors(atrias_msgs::robot_state &state) {
	// Don't fire another event if we're already in estop (or resetting).
	controllerManager::RtOpsCommand cur_state = rtOps->getStateMachine()->getRtOpsState();
	if (cur_state == controllerManager::RtOpsCommand::E_STOP ||
	    cur_state == controllerManager::RtOpsCommand::RESET)
		return;
	
	// Go through each Medulla, checking for the reset state:
	if (state.boomMedullaState        == medulla_state_error ||
	    state.lLeg.hipMedullaState    == medulla_state_error ||
	    state.lLeg.halfA.medullaState == medulla_state_error ||
	    state.lLeg.halfB.medullaState == medulla_state_error ||
	    state.rLeg.hipMedullaState    == medulla_state_error ||
	    state.rLeg.halfA.medullaState == medulla_state_error ||
	    state.rLeg.halfB.medullaState == medulla_state_error) {
		
		// Send an event... and eStop too (even though that'll happen anyway...)
		rtOps->getStateMachine()->eStop(controllerManager::RtOpsEvent::MEDULLA_ESTOP);
	}
}

}

}
