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
}

void BoomMedulla::processPitchEncoder(RTT::os::TimeService::nsecs deltaTime,
                                      atrias_msgs::robot_state& robotState) {
	
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
	
	robot_state.boomMedullaState = *state;
}

}

}
