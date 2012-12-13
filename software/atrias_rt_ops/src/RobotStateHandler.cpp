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
	newState.rtOpsState =
		(RtOpsState_t) rtOps->getStateMachine()->getRtOpsState();
	checkForNewErrors(newState);
	
	RTT::os::MutexLock lock(robotStateLock);
	robotState = newState;
}

void RobotStateHandler::checkForNewErrors(atrias_msgs::robot_state &state) {
	// Don't fire another event if we're already in estop (or resetting).
	RtOpsState cur_state = rtOps->getStateMachine()->getRtOpsState();
	if (cur_state == RtOpsState::E_STOP ||
	    cur_state == RtOpsState::RESET)
		return;
	
	// Go through each Medulla, checking for the reset state:
	if (state.boomMedullaState        == medulla_state_error ||
	    state.lLeg.hip.medullaState   == medulla_state_error ||
	    state.lLeg.halfA.medullaState == medulla_state_error ||
	    state.lLeg.halfB.medullaState == medulla_state_error ||
	    state.rLeg.hip.medullaState   == medulla_state_error ||
	    state.rLeg.halfA.medullaState == medulla_state_error ||
	    state.rLeg.halfB.medullaState == medulla_state_error) {
		
		// Send an event... and eStop too (even though that'll happen anyway...)
		rtOps->getStateMachine()->eStop(RtOpsEvent::MEDULLA_ESTOP);
	}
}

}

}

// vim: noexpandtab
