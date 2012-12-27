#include "atrias_rt_ops/StateMachine.h"

namespace atrias {

namespace rtOps {

StateMachine::StateMachine(RTOps* rt_ops) {
	rtOps         = rt_ops;
	controllerCmd = RtOpsState::DISABLED;
	state         = RtOpsState::DISABLED;
}

bool StateMachine::runningCommon(RtOpsState guiCmd) {
	if (rtOps->getSafety()->shouldEStop()) {
		RtOpsStateChangeMetadata metadata;
		metadata.newState = RtOpsState::E_STOP;
		metadata.reason   = RtOpsStateChgReason::ESTOP;
		setState(RtOpsState::E_STOP, buildEventMetadata(RtOpsEvent::RTOPS_STATE_CHG, metadata));
		return true;
	}

	if (guiCmd == RtOpsState::E_STOP) {
		setState(RtOpsState::E_STOP, buildEventMetadata(RtOpsEvent::GUI_STATE_CHG, RtOpsState::E_STOP));
		return true;
	}

	if (controllerCmd == RtOpsState::E_STOP) {
		setState(RtOpsState::E_STOP, buildEventMetadata(RtOpsEvent::CONT_STATE_CHG, RtOpsState::E_STOP));
		return true;
	}

	if (rtOps->getSafety()->shouldHalt()) {
		RtOpsStateChangeMetadata metadata;
		metadata.newState = RtOpsState::HALT;
		metadata.reason   = RtOpsStateChgReason::SAFETIES;
		setState(RtOpsState::HALT, buildEventMetadata(RtOpsEvent::RTOPS_STATE_CHG, metadata));
		return true;
	}

	if (controllerCmd == RtOpsState::HALT) {
		setState(RtOpsState::HALT, buildEventMetadata(RtOpsEvent::CONT_STATE_CHG, RtOpsState::HALT));
		return true;
	}

	return false;
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
			if (runningCommon(guiCmd))
				break;

			if (guiCmd == RtOpsState::SOFT_STOP ||
			    guiCmd == RtOpsState::STOP)
			{
				setState(guiCmd, buildEventMetadata(RtOpsEvent::GUI_STATE_CHG, guiCmd));
				break;
			}

			if (controllerCmd == RtOpsState::SOFT_STOP ||
			    controllerCmd == RtOpsState::STOP)
			{
				setState(controllerCmd, buildEventMetadata(RtOpsEvent::CONT_STATE_CHG, controllerCmd));
				break;
			}

			break;

		case RtOpsState::SOFT_STOP:
			if (runningCommon(guiCmd))
				break;

			if (guiCmd == RtOpsState::STOP) {
				setState(RtOpsState::STOP, buildEventMetadata(RtOpsEvent::GUI_STATE_CHG, RtOpsState::STOP));
				break;
			}

			if (controllerCmd != RtOpsState::SOFT_STOP) {
				setState(RtOpsState::STOP, buildEventMetadata(RtOpsEvent::CONT_STATE_CHG, RtOpsState::STOP));
				break;
			}

			break;

		case RtOpsState::STOP:
			if (runningCommon(guiCmd))
				break;

			if (!(rtOps->getRobotStateHandler()->getRobotState().medullasInRun)) {
				RtOpsStateChangeMetadata metadata;
				metadata.newState = RtOpsState::DISABLED;
				metadata.reason   = RtOpsStateChgReason::STOP_DONE;
				setState(RtOpsState::DISABLED, buildEventMetadata(RtOpsEvent::RTOPS_STATE_CHG, metadata));
				break;
			}

			break;

		case RtOpsState::RESET: {
			atrias_msgs::robot_state robotState = rtOps->getRobotStateHandler()->getRobotState();
			if (!robotState.medullasInRun   &&
			    !robotState.medullasInError &&
			    !robotState.medullasInHalt)
			{
				RtOpsStateChangeMetadata metadata;
				metadata.newState = RtOpsState::DISABLED;
				metadata.reason   = RtOpsStateChgReason::RESET_DONE;
				setState(RtOpsState::DISABLED, buildEventMetadata(RtOpsEvent::RTOPS_STATE_CHG, metadata));
				break;
			}
			break;
		}
		case RtOpsState::E_STOP:
			if (guiCmd == RtOpsState::RESET) {
				setState(RtOpsState::RESET, buildEventMetadata(RtOpsEvent::GUI_STATE_CHG, RtOpsState::RESET));
				break;
			}

			break;

		case RtOpsState::HALT:
			if (rtOps->getSafety()->shouldEStop()) {
				RtOpsStateChangeMetadata metadata;
				metadata.newState = RtOpsState::E_STOP;
				metadata.reason   = RtOpsStateChgReason::ESTOP;
				setState(RtOpsState::E_STOP, buildEventMetadata(RtOpsEvent::RTOPS_STATE_CHG, metadata));
			} else if (guiCmd == RtOpsState::E_STOP) {
				setState(RtOpsState::E_STOP, buildEventMetadata(RtOpsEvent::GUI_STATE_CHG, RtOpsState::E_STOP));
			} else if (controllerCmd == RtOpsState::E_STOP) {
				setState(RtOpsState::E_STOP, buildEventMetadata(RtOpsEvent::CONT_STATE_CHG, RtOpsState::E_STOP));
			}

			break;

		default: {
			// Oops. Shouldn't get here... Abort! Abort!
			RtOpsStateChangeMetadata metadata;
			metadata.newState = RtOpsState::E_STOP;
			metadata.reason   = RtOpsStateChgReason::ESTOP;
			setState(RtOpsState::E_STOP, buildEventMetadata(RtOpsEvent::RTOPS_STATE_CHG, metadata));
			break;
		}
	}
}

}

}

// vim: noexpandtab
