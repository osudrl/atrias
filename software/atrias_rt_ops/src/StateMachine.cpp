#include "atrias_rt_ops/StateMachine.h"

namespace atrias {

namespace rtOps {

StateMachine::StateMachine(RTOps* rt_ops) {
	rtOps        = rt_ops;
	resetCounter = 0;
	setState(RtOpsState::NO_CONTROLLER_LOADED);
}

void StateMachine::eStop(RtOpsEvent event) {
	setState(RtOpsState::E_STOP);
	rtOps->getOpsLogger()->sendEvent(event);
}

void StateMachine::setState(RtOpsState new_state) {
	RTT::os::MutexLock lock(currentStateLock);
	
	// Refuse to leave ESTOP state unless a reset is occurring.
	// This prevents a potential race condition between an ESTOP and a
	// controller manager command.
	if (currentState == RtOpsState::E_STOP &&
	    new_state    != RtOpsState::RESET)
		return;
	
	currentState = new_state;
}

void StateMachine::ackCMState(RtOpsState state) {
	switch (state) {
		case RtOpsState::NO_CONTROLLER_LOADED:
			rtOps->getOpsLogger()->sendEvent(RtOpsEvent::ACK_NO_CONTROLLER_LOADED);
			break;
			
		case RtOpsState::DISABLED:
			rtOps->getOpsLogger()->sendEvent(RtOpsEvent::ACK_DISABLE);
			break;
			
		case RtOpsState::ENABLED:
			rtOps->getOpsLogger()->sendEvent(RtOpsEvent::ACK_ENABLE);
			break;
			
		case RtOpsState::RESET:
			// Don't send an ack here... it happens after the reset is complete.
			break;
			
		case RtOpsState::E_STOP:
			rtOps->getOpsLogger()->sendEvent(RtOpsEvent::ACK_E_STOP);
			break;
		
		case RtOpsState::HALT:
			rtOps->getOpsLogger()->sendEvent(RtOpsEvent::ACK_HALT);
			break;
			
		default:
			// This shouldn't happen, because this should never be called with an
			// invalid RtOpsCommand. But the check's here anyway.
			eStop(RtOpsEvent::ACK_INVALID);
			break;
	}
}

medulla_state_t StateMachine::calcState(atrias_msgs::controller_output controllerOutput) {
	switch (getRtOpsState()) {
		case RtOpsState::E_STOP:
			return medulla_state_error;
			
		case RtOpsState::NO_CONTROLLER_LOADED:
			return medulla_state_idle;
			
		case RtOpsState::DISABLED:
			return medulla_state_idle;
			
		case RtOpsState::RESET:
			resetCounter++;
			if (resetCounter > MEDULLA_RESET_TIME_MS) {
				resetCounter = 0;
				setState(RtOpsState::NO_CONTROLLER_LOADED);
				rtOps->getOpsLogger()->sendEvent(RtOpsEvent::ACK_RESET);
			}
			return medulla_state_reset;
			
		case RtOpsState::ENABLED:
			if (controllerOutput.command == medulla_state_error)
				eStop(RtOpsEvent::CONTROLLER_ESTOP);

			/*
			if (rtOps->getSafety()->shouldEStop(controllerOutput)) {
				// This is a bit of a kludge -- send the MEDULLA_ESTOP event to tell the GUI and CM that
				// it's enterinng ESTOP state.
				eStop(RtOpsEvent::MEDULLA_ESTOP);
				return medulla_state_error;
			}
			
			if (rtOps->getSafety()->shouldHalt()) {
				setState(RtOpsState::HALT);
				return medulla_state_halt;
			}
			*/
			
			return (medulla_state_t) controllerOutput.command;
		
		case RtOpsState::HALT:
			return medulla_state_halt;
			
		default:
			eStop(RtOpsEvent::INVALID_RT_OPS_STATE);
			return medulla_state_error;
	}
}

void StateMachine::newCMState(RtOpsState new_state) {
	switch (new_state) {
		case RtOpsState::NO_CONTROLLER_LOADED:
			rtOps->getControllerLoop()->setControllerUnloaded();
			break;
			
		case RtOpsState::DISABLED:
			rtOps->getControllerLoop()->setControllerLoaded();
			break;
			
		case RtOpsState::ENABLED:
			/*
			if (rtOps->getSafety()->shouldHalt())
				new_state = RtOpsState::DISABLED;
			*/
			
			break;
			
		case RtOpsState::RESET:
			rtOps->getControllerLoop()->setControllerUnloaded();
			break;
			
		case RtOpsState::E_STOP:
			eStop(RtOpsEvent::CM_COMMAND_ESTOP);
			break;
		
		case RtOpsState::HALT:
			// Nothing special needs to be done here.
			break;
			
		default:
			eStop(RtOpsEvent::INVALID_CM_COMMAND);
			rtOps->getControllerLoop()->setControllerUnloaded();
			return; // Let's not set an invalid state or respond to this command.
	}
	setState(new_state);
	ackCMState(new_state);
}

RtOpsState StateMachine::getRtOpsState() {
	RTT::os::MutexLock lock(currentStateLock);
	return currentState;
}

}

}

// vim: noexpandtab
