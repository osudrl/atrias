#include "atrias_rt_ops/LegMedulla.h"

LegMedulla::LegMedulla(RTOps* rt_ops, ec_slavet* slave) :
            Medulla() {
	rtOps               = rt_ops;
	legPositionOffset   = 0.0;
	
	/* These are in the order of the PDO entries in the ESI XML file.
	 * I'd love cur_index to be a void * here, but C doesn't like doing pointer
	 * arithmetic w/ void pointers due to an unhelpful "feature" of the language
	 * (specifically, cur_index + 1 is equivalent to &(cur_index[1]) regardless
	 * of type...)
	 */
	uint8_t * cur_index = slave->outputs;
	
	setPdoPointer(cur_index, command);
	setPdoPointer(cur_index, counter);
	setPdoPointer(cur_index, motorCurrent);
	
	cur_index           = slave->inputs;
	
	setPdoPointer(cur_index, id);
	setPdoPointer(cur_index, state);
	setPdoPointer(cur_index, timingCounter);
	setPdoPointer(cur_index, errorFlags);
	setPdoPointer(cur_index, limitSwitch);
	setPdoPointer(cur_index, toeSensor);
	setPdoPointer(cur_index, motorEncoder);
	setPdoPointer(cur_index, motorEncoderTimestamp);
	setPdoPointer(cur_index, legEncoder);
	setPdoPointer(cur_index, legEncoderTimestamp);
	setPdoPointer(cur_index, incrementalEncoder);
	setPdoPointer(cur_index, incrementalEncoderTimestamp);
	setPdoPointer(cur_index, motorVoltage);
	setPdoPointer(cur_index, logicVoltage);
	setPdoPointer(cur_index, thermistor0);
	setPdoPointer(cur_index, thermistor1);
	setPdoPointer(cur_index, thermistor2);
	setPdoPointer(cur_index, thermistor3);
	setPdoPointer(cur_index, thermistor4);
	setPdoPointer(cur_index, thermistor5);
	setPdoPointer(cur_index, amp1MeasuredCurrent);
	setPdoPointer(cur_index, amp2MeasuredCurrent);
	
	motorEncoderValue                = (int64_t) *motorEncoder;
	motorEncoderTimestampValue       =           *motorEncoderTimestamp;
	legEncoderValue                  = (int64_t) *legEncoder;
	legEncoderTimestampValue         =           *legEncoderTimestamp;
	incrementalEncoderValue          =           *incrementalEncoder;
	incrementalEncoderTimestampValue =           *incrementalEncoderTimestamp;
	timingCounterValue               =           *timingCounter;
	processPositions();
	legPositionOffset                = calcLegPositionOffset();
}

double LegMedulla::calcLegPositionOffset() {
	double offset = 0.0;
	switch(*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			offset = rtOps->rtOpsCycle.robotState.lLeg.halfA.motorAngle - rtOps->rtOpsCycle.robotState.lLeg.halfA.legAngle;
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			offset = rtOps->rtOpsCycle.robotState.lLeg.halfB.motorAngle - rtOps->rtOpsCycle.robotState.lLeg.halfB.legAngle;
			break;
		/*case MEDULLA_RIGHT_LEG_A_ID:
			offset = rtOps->rtOpsCycle.robotState.rLeg.halfA.motorAngle - rtOps->rtOpsCycle.robotState.rLeg.halfA.legAngle;
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			offset = rtOps->rtOpsCycle.robotState.rLeg.halfB.motorAngle - rtOps->rtOpsCycle.robotState.rLeg.halfB.legAngle;
			break;*/
	}

	if (offset >  MAX_LEG_POS_ADJUSTMENT) {
		log(RTT::Warning) << "Spring deflection adjustment limit reached!" << endlog();
		return    MAX_LEG_POS_ADJUSTMENT;
	}
	if (offset < -MAX_LEG_POS_ADJUSTMENT) {
		log(RTT::Warning) << "Spring deflection adjustment limit reached!" << endlog();
		return   -MAX_LEG_POS_ADJUSTMENT;
	}
	return offset;
}

void LegMedulla::processIncrementalEncoders(RTT::os::TimeService::nsecs deltaTime) {
	// This compensates for wraparound.
	int16_t deltaPos = ((int32_t) *incrementalEncoder + (1 << 15) - incrementalEncoderValue) % (1 << 16) - (1 << 15);
	incrementalEncoderValue += deltaPos;
	
	// Let's take into account the timestamps, too.
	double adjustedTime = ((double) deltaTime) / SECOND_IN_NANOSECONDS +
	                      ((double) (*incrementalEncoderTimestamp - incrementalEncoderTimestampValue))
	                      / MEDULLA_TIMER_FREQ;
	
	incrementalEncoderTimestampValue = *incrementalEncoderTimestamp;
	
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			rtOps->rtOpsCycle.robotState.lLeg.halfA.rotorAngle += deltaPos * INC_ENC_RAD_PER_TICK;
			rtOps->rtOpsCycle.robotState.lLeg.halfA.rotorVelocity =
				((double) deltaPos) * INC_ENC_RAD_PER_TICK / adjustedTime;
			break;
			
		case MEDULLA_LEFT_LEG_B_ID:
			rtOps->rtOpsCycle.robotState.lLeg.halfB.rotorAngle += deltaPos * INC_ENC_RAD_PER_TICK;
			rtOps->rtOpsCycle.robotState.lLeg.halfB.rotorVelocity =
				((double) deltaPos) * INC_ENC_RAD_PER_TICK / adjustedTime;
			break;
			
		/*case MEDULLA_RIGHT_LEG_A_ID:
			rtOps->rtOpsCycle.robotState.rLeg.halfA.rotorAngle += deltaPos * INC_ENC_RAD_PER_TICK;
			rtOps->rtOpsCycle.robotState.rLeg.halfA.rotorVelocity =
				((double) deltaPos) * INC_ENC_RAD_PER_TICK / adjustedTime;
			break;
			
		case MEDULLA_RIGHT_LEG_B_ID:
			rtOps->rtOpsCycle.robotState.rLeg.halfB.rotorAngle += deltaPos * INC_ENC_RAD_PER_TICK;
			rtOps->rtOpsCycle.robotState.rLeg.halfB.rotorVelocity =
				((double) deltaPos) * INC_ENC_RAD_PER_TICK / adjustedTime;
			break;*/
	}
}

void LegMedulla::processReceiveData() {
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
	
	processPositions();
	processVelocities(deltaTime);
	processIncrementalEncoders(deltaTime);
	processThermistors();
	processLimitSwitches();
	processVoltages();
	processCurrents();
	
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			rtOps->cmOut.medullaStates[0]     = *state;
			//rtOps->cmOut.medullaErrorFlags[0] = *errorFlags;
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			rtOps->cmOut.medullaStates[1]     = *state;
			//rtOps->cmOut.medullaErrorFlags[1] = *errorFlags;
			break;
		/*case MEDULLA_RIGHT_LEG_A_ID:
			rtOps->cmOut.medullaStates[2]     = *state;
			//rtOps->cmOut.medullaErrorFlags[2] = *errorFlags;
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			rtOps->cmOut.medullaStates[3]     = *state;
			//rtOps->cmOut.medullaErrorFlags[3] = *errorFlags;
			break;*/
	}
}

int32_t LegMedulla::calcMotorCurrentOut() {
	// Don't command any amount of torque if we're not enabled.
	if (state_command != medulla_state_run) return 0;
	
	// If the ID isn't recognized, command 0 torque.
	double torqueCmd = 0.0;
	
	switch(*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			torqueCmd = rtOps->controllerOutput.lLeg.motorCurrentA;
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			torqueCmd = rtOps->controllerOutput.lLeg.motorCurrentB;
			break;
		/*case MEDULLA_RIGHT_LEG_A_ID:
			torqueCmd = rtOps->controllerOutput.rLeg.motorCurrentA;
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			torqueCmd = rtOps->controllerOutput.rLeg.motorCurrentB;
			break;*/
	}

	if (torqueCmd > MAX_MTR_TRQ_CMD)
		torqueCmd = MAX_MTR_TRQ_CMD;
	if (torqueCmd < MIN_MTR_TRQ_CMD)
		torqueCmd = MIN_MTR_TRQ_CMD;
	
	return (int32_t) (((double) MTR_MAX_COUNT) * torqueCmd / MTR_MAX_TORQUE);
}

inline double LegMedulla::encTicksToRad(uint32_t ticks, uint32_t calib_val, double rad_per_tick, double calib_val_rad) {
	// Be careful of integer/double conversions here.
	return calib_val_rad + ((double) (((int64_t) ticks) - ((int64_t) calib_val))) * rad_per_tick;
}

void LegMedulla::processPositions() {
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			rtOps->rtOpsCycle.robotState.lLeg.halfA.motorAngle =
				encTicksToRad(*motorEncoder, LEFT_TRAN_A_CALIB_VAL,  LEFT_TRAN_A_RAD_PER_CNT, LEG_A_CALIB_LOC);
			rtOps->rtOpsCycle.robotState.lLeg.halfA.legAngle   =
				encTicksToRad(*legEncoder,   LEFT_LEG_A_CALIB_VAL,   LEFT_LEG_A_RAD_PER_CNT,  LEG_A_CALIB_LOC + legPositionOffset);
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			rtOps->rtOpsCycle.robotState.lLeg.halfB.motorAngle =
				encTicksToRad(*motorEncoder, LEFT_TRAN_B_CALIB_VAL,  LEFT_TRAN_B_RAD_PER_CNT, LEG_B_CALIB_LOC);
			rtOps->rtOpsCycle.robotState.lLeg.halfB.legAngle   =
				encTicksToRad(*legEncoder,   LEFT_LEG_B_CALIB_VAL,   LEFT_LEG_B_RAD_PER_CNT,  LEG_B_CALIB_LOC + legPositionOffset);
			break;
		/*case MEDULLA_RIGHT_LEG_A_ID:
			rtOps->rtOpsCycle.robotState.rLeg.halfA.motorAngle =
				encTicksToRad(*motorEncoder, RIGHT_TRAN_A_CALIB_VAL, RIGHT_TRAN_A_RAD_PER_CNT, LEG_A_CALIB_LOC);
			rtOps->rtOpsCycle.robotState.rLeg.halfA.legAngle   =
				encTicksToRad(*legEncoder,   RIGHT_LEG_A_CALIB_VAL,  RIGHT_LEG_A_RAD_PER_CNT,  LEG_A_CALIB_LOC + legPositionOffset);
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			rtOps->rtOpsCycle.robotState.rLeg.halfB.motorAngle =
				encTicksToRad(*motorEncoder, RIGHT_TRAN_B_CALIB_VAL, RIGHT_TRAN_B_RAD_PER_CNT, LEG_B_CALIB_LOC);
			rtOps->rtOpsCycle.robotState.rLeg.halfB.legAngle   =
				encTicksToRad(*legEncoder,   RIGHT_LEG_B_CALIB_VAL,  RIGHT_LEG_B_RAD_PER_CNT,  LEG_B_CALIB_LOC + legPositionOffset);
			break;*/
	}
}

void LegMedulla::processVelocities(RTT::os::TimeService::nsecs deltaTime) {
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			// The division by 32 million translates timer ticks from the microcontroller into seconds
			rtOps->rtOpsCycle.robotState.lLeg.halfA.motorVelocity =
				((double) (((int64_t) *motorEncoder) - motorEncoderValue)) * LEFT_TRAN_A_RAD_PER_CNT  /
				(((double) deltaTime) / 1000000000.0 + ((double) (*motorEncoderTimestamp - motorEncoderTimestampValue)) / MEDULLA_TIMER_FREQ);
			rtOps->rtOpsCycle.robotState.lLeg.halfA.legVelocity   =
				((double) (((int64_t) *legEncoder)   - legEncoderValue))   * LEFT_LEG_A_RAD_PER_CNT   /
				(((double) deltaTime) / 1000000000.0 + ((double) (*legEncoderTimestamp   - legEncoderTimestampValue))   / MEDULLA_TIMER_FREQ);
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			rtOps->rtOpsCycle.robotState.lLeg.halfB.motorVelocity =
				((double) (((int64_t) *motorEncoder) - motorEncoderValue)) * LEFT_TRAN_B_RAD_PER_CNT  /
				(((double) deltaTime) / 1000000000.0 + ((double) (*motorEncoderTimestamp - motorEncoderTimestampValue)) / MEDULLA_TIMER_FREQ);
			rtOps->rtOpsCycle.robotState.lLeg.halfB.legVelocity   =
				((double) (((int64_t) *legEncoder)   - legEncoderValue))   * LEFT_LEG_B_RAD_PER_CNT   /
				(((double) deltaTime) / 1000000000.0 + ((double) (*legEncoderTimestamp   -   legEncoderTimestampValue)) / MEDULLA_TIMER_FREQ);
			break;
		/*case MEDULLA_RIGHT_LEG_A_ID:
			rtOps->rtOpsCycle.robotState.rLeg.halfA.motorVelocity =
				((double) (((int64_t) *motorEncoder) - motorEncoderValue)) * RIGHT_TRAN_A_RAD_PER_CNT /
				(((double) deltaTime) / 1000000000.0 + ((double) (*motorEncoderTimestamp - motorEncoderTimestampValue)) / MEDULLA_TIMER_FREQ);
			rtOps->rtOpsCycle.robotState.rLeg.halfA.legVelocity   =
				((double) (((int64_t) *legEncoder)   - legEncoderValue))   * RIGHT_LEG_B_RAD_PER_CNT  /
				(((double) deltaTime) / 1000000000.0 + ((double) (*legEncoderTimestamp   - legEncoderTimestampValue))   / MEDULLA_TIMER_FREQ);
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			rtOps->rtOpsCycle.robotState.rLeg.halfB.motorVelocity =
				((double) (((int64_t) *motorEncoder) - motorEncoderValue)) * RIGHT_TRAN_B_RAD_PER_CNT /
				(((double) deltaTime) / 1000000000.0 + ((double) (*motorEncoderTimestamp - motorEncoderTimestampValue)) / MEDULLA_TIMER_FREQ);
			rtOps->rtOpsCycle.robotState.rLeg.halfB.legVelocity   =
				((double) (((int64_t) *legEncoder)   - legEncoderValue))   * RIGHT_LEG_B_RAD_PER_CNT  /
				(((double) deltaTime) / 1000000000.0 + ((double) (*legEncoderTimestamp   -   legEncoderTimestampValue)) / MEDULLA_TIMER_FREQ);
			break;*/
	}

	motorEncoderValue          = (int64_t) *motorEncoder;
	motorEncoderTimestampValue = *motorEncoderTimestamp;
	legEncoderValue            = (int64_t) *legEncoder;
	legEncoderTimestampValue   = *legEncoderTimestamp;
}

void LegMedulla::processThermistors() {
	/*switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			rtOps->rtOpsCycle.robotState.lLeg.halfA.motorThermA1 = processThermistorValue(*thermistor0);
			rtOps->rtOpsCycle.robotState.lLeg.halfA.motorThermA2 = processThermistorValue(*thermistor1);
			rtOps->rtOpsCycle.robotState.lLeg.halfA.motorThermB1 = processThermistorValue(*thermistor2);
			rtOps->rtOpsCycle.robotState.lLeg.halfA.motorThermB2 = processThermistorValue(*thermistor3);
			rtOps->rtOpsCycle.robotState.lLeg.halfA.motorThermC1 = processThermistorValue(*thermistor4);
			rtOps->rtOpsCycle.robotState.lLeg.halfA.motorThermC2 = processThermistorValue(*thermistor5);
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			rtOps->rtOpsCycle.robotState.lLeg.halfB.motorThermA1 = processThermistorValue(*thermistor0);
			rtOps->rtOpsCycle.robotState.lLeg.halfB.motorThermA2 = processThermistorValue(*thermistor1);
			rtOps->rtOpsCycle.robotState.lLeg.halfB.motorThermB1 = processThermistorValue(*thermistor2);
			rtOps->rtOpsCycle.robotState.lLeg.halfB.motorThermB2 = processThermistorValue(*thermistor3);
			rtOps->rtOpsCycle.robotState.lLeg.halfB.motorThermC1 = processThermistorValue(*thermistor4);
			rtOps->rtOpsCycle.robotState.lLeg.halfB.motorThermC2 = processThermistorValue(*thermistor5);
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			rtOps->rtOpsCycle.robotState.rLeg.halfA.motorThermA1 = processThermistorValue(*thermistor0);
			rtOps->rtOpsCycle.robotState.rLeg.halfA.motorThermA2 = processThermistorValue(*thermistor1);
			rtOps->rtOpsCycle.robotState.rLeg.halfA.motorThermB1 = processThermistorValue(*thermistor2);
			rtOps->rtOpsCycle.robotState.rLeg.halfA.motorThermB2 = processThermistorValue(*thermistor3);
			rtOps->rtOpsCycle.robotState.rLeg.halfA.motorThermC1 = processThermistorValue(*thermistor4);
			rtOps->rtOpsCycle.robotState.rLeg.halfA.motorThermC2 = processThermistorValue(*thermistor5);
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			rtOps->rtOpsCycle.robotState.rLeg.halfB.motorThermA1 = processThermistorValue(*thermistor0);
			rtOps->rtOpsCycle.robotState.rLeg.halfB.motorThermA2 = processThermistorValue(*thermistor1);
			rtOps->rtOpsCycle.robotState.rLeg.halfB.motorThermB1 = processThermistorValue(*thermistor2);
			rtOps->rtOpsCycle.robotState.rLeg.halfB.motorThermB2 = processThermistorValue(*thermistor3);
			rtOps->rtOpsCycle.robotState.rLeg.halfB.motorThermC1 = processThermistorValue(*thermistor4);
			rtOps->rtOpsCycle.robotState.rLeg.halfB.motorThermC2 = processThermistorValue(*thermistor5);
			break;
	}*/
}

void LegMedulla::processCurrents() {
/*	double current1 = processAmplifierCurrent(*amp1MeasuredCurrent);
	double current2 = processAmplifierCurrent(*amp2MeasuredCurrent);
	
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			rtOps->rtOpsCycle.robotState.lLeg.halfA.amp1Current  = current1;
			rtOps->rtOpsCycle.robotState.lLeg.halfA.amp2Current  = current2;
			rtOps->rtOpsCycle.robotState.lLeg.halfA.motorCurrent = current1 + current2;
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			rtOps->rtOpsCycle.robotState.lLeg.halfB.amp1Current  = current1;
			rtOps->rtOpsCycle.robotState.lLeg.halfB.amp2Current  = current2;
			rtOps->rtOpsCycle.robotState.lLeg.halfB.motorCurrent = current1 + current2;
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			rtOps->rtOpsCycle.robotState.rLeg.halfA.amp1Current  = current1;
			rtOps->rtOpsCycle.robotState.rLeg.halfA.amp2Current  = current2;
			rtOps->rtOpsCycle.robotState.rLeg.halfA.motorCurrent = current1 + current2;
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			rtOps->rtOpsCycle.robotState.rLeg.halfB.amp1Current  = current1;
			rtOps->rtOpsCycle.robotState.rLeg.halfB.amp2Current  = current2;
			rtOps->rtOpsCycle.robotState.rLeg.halfB.motorCurrent = current1 + current2;
			break;
	}*/
}

void LegMedulla::processLimitSwitches() {
	// TODO: figure out which limit switches are which and implement this.
}

void LegMedulla::processVoltages() {
	/*switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			rtOps->rtOpsCycle.robotState.lLeg.halfA.motorVoltage = processADCValue(*motorVoltage)*30.0;
			rtOps->rtOpsCycle.robotState.lLeg.halfA.logicVoltage = processADCValue(*logicVoltage)*6.0;
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			rtOps->rtOpsCycle.robotState.lLeg.halfB.motorVoltage = processADCValue(*motorVoltage);
			rtOps->rtOpsCycle.robotState.lLeg.halfB.logicVoltage = processADCValue(*logicVoltage);
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			rtOps->rtOpsCycle.robotState.rLeg.halfA.motorVoltage = processADCValue(*motorVoltage);
			rtOps->rtOpsCycle.robotState.rLeg.halfA.logicVoltage = processADCValue(*logicVoltage);
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			rtOps->rtOpsCycle.robotState.rLeg.halfB.motorVoltage = processADCValue(*motorVoltage);
			rtOps->rtOpsCycle.robotState.rLeg.halfB.logicVoltage = processADCValue(*logicVoltage);
			break;
	}*/
}

void LegMedulla::processTransmitData() {
	*counter      = ++local_counter;
	*command      = state_command;
	*motorCurrent = calcMotorCurrentOut();
}

uint8_t LegMedulla::getID() {
	return *id;
}
