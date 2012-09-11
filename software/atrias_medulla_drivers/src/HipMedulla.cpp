#include "atrias_medulla_drivers/HipMedulla.h"

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
	
	setPdoPointer(cur_index, hipEncoder);
	setPdoPointer(cur_index, hipEncoderTimestamp);
	
	setPdoPointer(cur_index, motorVoltage);
	setPdoPointer(cur_index, logicVoltage);
	
	setPdoPointer(cur_index, thermistor0);
	setPdoPointer(cur_index, thermistor1);
	setPdoPointer(cur_index, thermistor2);
	
	setPdoPointer(cur_index, ampMeasuredCurrent);
	
	setPdoPointer(cur_index, accelX);
	setPdoPointer(cur_index, accelY);
	setPdoPointer(cur_index, accelZ);
	setPdoPointer(cur_index, angRateX);
	setPdoPointer(cur_index, angRateY);
	setPdoPointer(cur_index, angRateZ);
	setPdoPointer(cur_index, m11);
	setPdoPointer(cur_index, m12);
	setPdoPointer(cur_index, m13);
	setPdoPointer(cur_index, m21);
	setPdoPointer(cur_index, m22);
	setPdoPointer(cur_index, m23);
	setPdoPointer(cur_index, m31);
	setPdoPointer(cur_index, m32);
	setPdoPointer(cur_index, m33);
	setPdoPointer(cur_index, timer);
	
	setPdoPointer(cur_index, incrementalEncoder);
	setPdoPointer(cur_index, incrementalEncoderTimestamp);
	
	timingCounterValue               = *timingCounter;
	incrementalEncoderValue          = *incrementalEncoder;
	incrementalEncoderTimestampValue = *incrementalEncoderTimestamp;
	incrementalEncoderInitialized    = false;
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

void HipMedulla::updateLimitSwitches(atrias_msgs::robot_state_hip& hip) {
	hip.InsideLimitSwitch  = *limitSwitches & (1 << 0);
	hip.OutsideLimitSwitch = *limitSwitches & (1 << 1);
}

void HipMedulla::updateEncoderValues(RTT::os::TimeService::nsecs delta_time,
                                     atrias_msgs::robot_state_hip& hip) {
	double actualDeltaTime =
		((double) delta_time) / ((double) SECOND_IN_NANOSECONDS) +
		((double) (((int16_t) *incrementalEncoderTimestamp) -
		incrementalEncoderTimestampValue)) / MEDULLA_TIMER_FREQ;
	
	int16_t deltaPos = ((int32_t) *incrementalEncoder + (1 << 15) -
	                   incrementalEncoderValue) % (1 << 16) - (1 << 15);
	
	incrementalEncoderValue         += deltaPos;
	incrementalEncoderTimestampValue = *incrementalEncoderTimestamp;
	
	int dir = (*id == MEDULLA_LEFT_HIP_ID) ? LEFT_MOTOR_HIP_DIRECTION
	          : RIGHT_MOTOR_HIP_DIRECTION;
	int32_t calib_val = (*id == MEDULLA_LEFT_HIP_ID) ? LEFT_HIP_CALIB_VAL
	                    : RIGHT_HIP_CALIB_VAL;
	double  calib_pos = (*id == MEDULLA_LEFT_HIP_ID) ? LEFT_HIP_CALIB_POS
	                    : RIGHT_HIP_CALIB_POS;
	
	hip.legBodyVelocity   = dir * HIP_INC_ENCODER_RAD_PER_TICK * deltaPos /
	                        actualDeltaTime;
	hip.legBodyAngle     += dir * HIP_INC_ENCODER_RAD_PER_TICK * deltaPos;
	hip.absoluteBodyAngle = (((int32_t) *hipEncoder) - calib_val) *
	                        HIP_ABS_ENCODER_RAD_PER_TICK + calib_pos;
	hip.legBodyAngle     += (hip.absoluteBodyAngle - hip.legBodyAngle) / 100000.0;
	if (!incrementalEncoderInitialized) {
		hip.legBodyAngle = hip.absoluteBodyAngle;
		incrementalEncoderInitialized = true;
	}
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
	
	atrias_msgs::robot_state_hip* hip_ptr;
	switch(*id) {
		case MEDULLA_LEFT_HIP_ID:
			hip_ptr = &(robot_state.lLeg.hip);
			break;
		case MEDULLA_RIGHT_HIP_ID:
			hip_ptr = &(robot_state.rLeg.hip);
			break;
		default:
			return;
	}
	
	atrias_msgs::robot_state_hip& hip = *hip_ptr;
	
	updateLimitSwitches(hip);
	updateEncoderValues(deltaTime, hip);
	
	hip.medullaState = *state;
	hip.errorFlags   = *errorFlags;
	hip.motorVoltage = decodeMotorVoltage(*motorVoltage);
	hip.logicVoltage = decodeLogicVoltage(*logicVoltage);
	hip.motorThermA  = processThermistorValue(*thermistor0);
	hip.motorThermB  = processThermistorValue(*thermistor1);
	hip.motorThermC  = processThermistorValue(*thermistor2);
	hip.motorCurrent = processAmplifierCurrent(*ampMeasuredCurrent);
	hip.accelX       = *accelX;
	hip.accelY       = *accelY;
	hip.accelZ       = *accelZ;
	hip.angRateX     = *angRateX;
	hip.angRateY     = *angRateY;
	hip.angRateZ     = *angRateZ;
	hip.m11          = *m11;
	hip.m12          = *m12;
	hip.m13          = *m13;
	hip.m21          = *m21;
	hip.m22          = *m22;
	hip.m23          = *m23;
	hip.m31          = *m31;
	hip.m32          = *m32;
	hip.m33          = *m33;
	hip.IMUTimer     = *timer;
}

}

}
