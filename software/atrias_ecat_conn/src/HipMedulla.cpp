#include "atrias_ecat_conn/HipMedulla.h"

namespace atrias {

namespace ecatConn {

HipMedulla::HipMedulla(uint8_t* inputs, uint8_t* outputs) : Medulla() {
	uint8_t* cur_index = outputs;
	
	setPdoPointer(cur_index, command);
	setPdoPointer(cur_index, counter);
	setPdoPointer(cur_index, motorCurrent);
	
	cur_index = inputs;
	
	setPdoPointer(cur_index, id);
	setPdoPointer(cur_index, state);
	setPdoPointer(cur_index, timingCounter);
	setPdoPointer(cur_index, errorFlags);
	setPdoPointer(cur_index, limitSwitches);
	
	timingCounterValue = *timingCounter;
}

int32_t HipMedulla::calcMotorCurrentOut(atrias_msgs::controller_output& controllerOutput) {
        // If the ID isn't recognized, command 0 torque.
        double torqueCmd = 0.0;
        
        switch(*id) {
                case MEDULLA_LEFT_HIP_ID:
                        torqueCmd = controllerOutput.lLeg.motorCurrentHip *
                        	LEFT_MOTOR_HIP_DIRECTION;
                        break;
                case MEDULLA_RIGHT_HIP_ID:
                        torqueCmd = controllerOutput.rLeg.motorCurrentHip *
                        	RIGHT_MOTOR_HIP_DIRECTION;
                        break;
        }
        
        return (int32_t) (((double) MTR_MAX_COUNT) * torqueCmd / MTR_HIP_MAX_TORQUE);
}

uint8_t HipMedulla::getID() {
	return *id;
}

void HipMedulla::processTransmitData(atrias_msgs::controller_output& controller_output) {
	*counter      = ++local_counter;
	*command      = controller_output.command;
	*motorCurrent = calcMotorCurrentOut(controller_output);
}

void HipMedulla::processReceiveData(atrias_msgs::robot_state& robot_state) {
	// If we don't have new data, don't run. It's pointless, and results in
	// NaN velocities.
	if (*timingCounter == timingCounterValue)
		return;
	// Calculate how much time has elapsed since the previous sensor readings.
	// Note: % isn't actually a modulo, hence the additional 256.
	RTT::os::TimeService::nsecs deltaTime =
		((((int16_t) *timingCounter) + 256 - ((int16_t) timingCounterValue)) % 256)
		* CONTROLLER_LOOP_PERIOD_NS;
	timingCounterValue = *timingCounter;
	
	atrias_msgs::robot_state_hip hip;
	switch(*id) {
		case MEDULLA_LEFT_HIP_ID:
			hip = robot_state.lLeg.hip;
			break;
		case MEDULLA_RIGHT_HIP_ID:
			hip = robot_state.rLeg.hip;
			break;
		default:
			return;
	}
	hip.medullaState = *state;
	hip.errorFlags   = *errorFlags;
}

}

}
