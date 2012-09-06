#include "atrias_ecat_conn/BoomMedulla.h"

namespace atrias {

namespace ecatConn {

BoomMedulla::BoomMedulla(uint8_t* inputs, uint8_t* outputs) : Medulla() {
	uint8_t* cur_index = outputs;
	
	setPdoPointer(cur_index, command);
	setPdoPointer(cur_index, counter);
	
	cur_index = inputs;
	
	setPdoPointer(cur_index, id);
	setPdoPointer(cur_index, state);
	setPdoPointer(cur_index, timingCounter);
	setPdoPointer(cur_index, errorFlags);
	
	setPdoPointer(cur_index, xEncoder);
	setPdoPointer(cur_index, xTimestamp);
	
	setPdoPointer(cur_index, pitchEncoder);
	setPdoPointer(cur_index, pitchTimestamp);
	
	setPdoPointer(cur_index, zEncoder);
	setPdoPointer(cur_index, zTimestamp);
	
	setPdoPointer(cur_index, logicVoltage);
	
	pitchEncoderPos = (*pitchEncoder - BOOM_PITCH_VERTICAL_VALUE)
	                  % (1 << BOOM_ENCODER_BITS);
	
	// Compensate for the difference between % and modulo
	pitchEncoderPos += 1 << BOOM_ENCODER_BITS;
	
	// Compensate for wraparound.
	pitchEncoderPos = (pitchEncoderPos + (1 << (BOOM_ENCODER_BITS - 1))) %
	                  (1 << BOOM_ENCODER_BITS) - (1 << (BOOM_ENCODER_BITS - 1));
	
	pitchEncoderValue   = *pitchEncoder;
	pitchTimestampValue = *pitchTimestamp;
}

uint8_t BoomMedulla::getID() {
	return *id;
}

void BoomMedulla::processPitchEncoder(RTT::os::TimeService::nsecs deltaTime,
                                      atrias_msgs::robot_state& robotState) {
	// Obtain the delta
	int deltaPos = ((int32_t) *pitchEncoder) - ((int32_t) pitchEncoderValue);
	
	// Compensate for the difference between the % operator and the modulo operation.
	deltaPos    += 1 << BOOM_ENCODER_BITS;
	
	// Compensate for rollover
	deltaPos     = (deltaPos + (1 << (BOOM_ENCODER_BITS - 1))) %
	               (1 << BOOM_ENCODER_BITS) - (1 << (BOOM_ENCODER_BITS - 1));
	
	pitchEncoderPos += deltaPos;
	
	robotState.position.bodyPitch = pitchEncoderPos * PITCH_ENCODER_RAD_PER_TICK -
	                                M_PI / 2.0;
	
	robotState.position.bodyPitchVelocity =
		deltaPos * PITCH_ENCODER_RAD_PER_TICK /
		(((double) deltaTime) / SECOND_IN_NANOSECONDS + ((double)
		(((int16_t) *pitchTimestamp) - pitchTimestampValue)
		) / MEDULLA_TIMER_FREQ);
	
	pitchEncoderValue   = *pitchEncoder;
	pitchTimestampValue = *pitchTimestamp;
}

void BoomMedulla::processZEncoder(RTT::os::TimeService::nsecs deltaTime,
                                  atrias_msgs::robot_state&   robotState) {
	
}

void BoomMedulla::processReceiveData(atrias_msgs::robot_state& robot_state) {
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
	
	processPitchEncoder(deltaTime, robot_state);
	processZEncoder(    deltaTime, robot_state);
	
	robot_state.boomMedullaState      = *state;
	robot_state.boomMedullaErrorFlags = *errorFlags;
	robot_state.boomLogicVoltage      = decodeLogicVoltage(*logicVoltage);
}

}

}
