#include "atrias_rt_ops/StateMachine.h"

namespace atrias {

namespace rtOps {

StateMachine::StateMachine(RTOps* rt_ops) {
	rtOps        = rt_ops;
	setState(RtOpsState::DISABLED);
}

void StateMachine::setState(RtOpsState new_state) {
	RTT::os::MutexLock lock(stateLock);
	state = new_state;
}

RtOpsState StateMachine::getState() {
	RTT::os::MutexLock lock(stateLock);
	return state;
}

medulla_state_t StateMachine::calcMedullaCmd(atrias_msgs::controller_output &controller_output) {
	return medulla_state_error; // not implemented yet.
}

void StateMachine::run() {
	
}

}

}

// vim: noexpandtab
