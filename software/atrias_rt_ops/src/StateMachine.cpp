#include "atrias_rt_ops/StateMachine.h"

namespace atrias {

namespace rtOps {

StateMachine::StateMachine(RTOps* rt_ops) {
	rtOps         = rt_ops;
	controllerCmd = RtOpsState::DISABLED;
	state         = RtOpsState::DISABLED;
}

void StateMachine::setState(RtOpsState new_state, atrias_msgs::rt_ops_event event) {
	rtOps->sendEvent(event);
	RTT::os::MutexLock lock(stateLock);
	state = new_state;
}

RtOpsState StateMachine::getState() {
	RTT::os::MutexLock lock(stateLock);
	return state;
}

medulla_state_t StateMachine::calcMedullaCmd(RtOpsState controller_cmd) {
	controllerCmd = controller_cmd;
	switch (getState()) {
		case RtOpsState::DISABLED:
			return medulla_state_idle;

		case RtOpsState::ENABLED: // Intentional fallthrough
		case RtOpsState::SOFT_STOP:
			if (controllerCmd == RtOpsState::E_STOP)
				return medulla_state_error;

			if (controllerCmd == RtOpsState::HALT)
				return medulla_state_halt;

			return medulla_state_run;

		case RtOpsState::STOP:
			if (controllerCmd == RtOpsState::STOP) 
				return medulla_state_run;
			else
				return medulla_state_idle;

		case RtOpsState::RESET: {
			atrias_msgs::robot_state robotState = rtOps->getRobotStateHandler()->getRobotState();
			if (robotState.medullasInError || robotState.medullasInHalt)
				return medulla_state_reset;
			else
				return medulla_state_idle;
		}
		case RtOpsState::E_STOP:
			return medulla_state_error;

		case RtOpsState::HALT:
			return medulla_state_halt;

		default:
			// Oops, we've messed up. Let's limit the damage.
			return medulla_state_error;
	}
}

void StateMachine::run() {
	RtOpsState guiCmd = rtOps->getGuiComms()->getGuiCmd();
	switch (getState()) {
		case RtOpsState::DISABLED:
			if (guiCmd        == RtOpsState::ENABLED &&
			    controllerCmd == RtOpsState::ENABLED)
			{
				setState(RtOpsState::ENABLED, buildEventMetadata(RtOpsEvent::GUI_STATE_CHG, RtOpsState::ENABLED));
			}
			break;

		case RtOpsState::ENABLED:
			break;

		case RtOpsState::SOFT_STOP:
			break;

		case RtOpsState::STOP:
			break;

		case RtOpsState::RESET:
			break;

		case RtOpsState::E_STOP:
			break;

		case RtOpsState::HALT:
			break;

		default:
			// Oops. Shouldn't get here... Abort! Abort!
			break;
	}
}

}

}

// vim: noexpandtab
