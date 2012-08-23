#include "atrias_rt_ops/StateMachine.h"

namespace atrias {

namespace rtOps {

StateMachine::StateMachine(RTOps* rt_ops) {
	rtOps        = rt_ops;
	resetCounter = 0;
	setState(controllerManager::RtOpsCommand::NO_CONTROLLER_LOADED);
}

void StateMachine::eStop(controllerManager::RtOpsEvent event) {
	setState(controllerManager::RtOpsCommand::E_STOP);
	rtOps->getOpsLogger()->sendEvent(event);
}

void StateMachine::setState(controllerManager::RtOpsCommand new_state) {
	RTT::os::MutexLock lock(currentStateLock);
	
	// Refuse to leave ESTOP state unless a reset is occurring.
	// This prevents a potential race condition between an ESTOP and a
	// controller manager command.
	if (currentState == controllerManager::RtOpsCommand::E_STOP &&
	    new_state    != controllerManager::RtOpsCommand::RESET)
		return;
	
	currentState = new_state;
}

void StateMachine::ackCMState(controllerManager::RtOpsCommand state) {
	switch (state) {
		case controllerManager::RtOpsCommand::NO_CONTROLLER_LOADED:
			rtOps->getOpsLogger()->sendEvent(controllerManager::RtOpsEvent::ACK_NO_CONTROLLER_LOADED);
			break;
			
		case controllerManager::RtOpsCommand::DISABLE:
			rtOps->getOpsLogger()->sendEvent(controllerManager::RtOpsEvent::ACK_DISABLE);
			break;
			
		case controllerManager::RtOpsCommand::ENABLE:
			rtOps->getOpsLogger()->sendEvent(controllerManager::RtOpsEvent::ACK_ENABLE);
			break;
			
		case controllerManager::RtOpsCommand::RESET:
			rtOps->getOpsLogger()->sendEvent(controllerManager::RtOpsEvent::ACK_RESET);
			break;
			
		case controllerManager::RtOpsCommand::E_STOP:
			rtOps->getOpsLogger()->sendEvent(controllerManager::RtOpsEvent::ACK_E_STOP);
			break;
			
		default:
			// This shouldn't happen, because this should never be called with an
			// invalid RtOpsCommand. But the check's here anyway.
			eStop(controllerManager::RtOpsEvent::ACK_INVALID);
			break;
	}
}

medulla_state_t StateMachine::calcState(atrias_msgs::controller_output controllerOutput) {
	switch (getRtOpsState()) {
		case controllerManager::RtOpsCommand::E_STOP:
			return medulla_state_error;
			
		case controllerManager::RtOpsCommand::NO_CONTROLLER_LOADED:
			return medulla_state_idle;
			
		case controllerManager::RtOpsCommand::DISABLE:
			return medulla_state_idle;
			
		case controllerManager::RtOpsCommand::RESET:
			resetCounter++;
			if (resetCounter > MEDULLA_RESET_TIME_MS) {
				resetCounter = 0;
				setState(controllerManager::RtOpsCommand::NO_CONTROLLER_LOADED);
			}
			return medulla_state_reset;
			
		case controllerManager::RtOpsCommand::ENABLE:
			if (controllerOutput.command == medulla_state_error)
				eStop(controllerManager::RtOpsEvent::CONTROLLER_ESTOP);
			
			return (medulla_state_t) controllerOutput.command;
			
		default:
			eStop(controllerManager::RtOpsEvent::INVALID_RT_OPS_STATE);
			return medulla_state_error;
	}
}

void StateMachine::newCMState(controllerManager::RtOpsCommand new_state) {
	switch (new_state) {
		case controllerManager::RtOpsCommand::NO_CONTROLLER_LOADED:
			rtOps->getControllerLoop()->setControllerUnloaded();
			break;
			
		case controllerManager::RtOpsCommand::DISABLE:
			rtOps->getControllerLoop()->setControllerLoaded();
			break;
			
		case controllerManager::RtOpsCommand::ENABLE:
			break;
			
		case controllerManager::RtOpsCommand::RESET:
			rtOps->getControllerLoop()->setControllerUnloaded();
			break;
			
		case controllerManager::RtOpsCommand::E_STOP:
			eStop(controllerManager::RtOpsEvent::CM_COMMAND_ESTOP);
			break;
			
		default:
			eStop(controllerManager::RtOpsEvent::INVALID_CM_COMMAND);
			rtOps->getControllerLoop()->setControllerUnloaded();
			return; // Let's not set an invalid state or respond to this command.
	}
	setState(new_state);
	ackCMState(new_state);
}

controllerManager::RtOpsCommand StateMachine::getRtOpsState() {
	RTT::os::MutexLock lock(currentStateLock);
	return currentState;
}

}

}
