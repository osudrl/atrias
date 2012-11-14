#include "atrias_medulla_drivers/LegMedulla.h"

namespace atrias {

namespace medullaDrivers {

LegMedulla::LegMedulla() : Medulla() {
	pdoEntryDatas[0]  = {1, (void**) &command};
	pdoEntryDatas[1]  = {2, (void**) &counter};
	pdoEntryDatas[2]  = {4, (void**) &motorCurrent};
	pdoEntryDatas[3]  = {1, (void**) &id};
	pdoEntryDatas[4]  = {1, (void**) &state};
	pdoEntryDatas[5]  = {1, (void**) &timingCounter};
	pdoEntryDatas[6]  = {1, (void**) &errorFlags};
	pdoEntryDatas[7]  = {1, (void**) &limitSwitch};
	pdoEntryDatas[8]  = {2, (void**) &toeSensor};
	pdoEntryDatas[9]  = {4, (void**) &motorEncoder};
	pdoEntryDatas[10] = {2, (void**) &motorEncoderTimestamp};
	pdoEntryDatas[11] = {4, (void**) &legEncoder};
	pdoEntryDatas[12] = {2, (void**) &legEncoderTimestamp};
	pdoEntryDatas[13] = {2, (void**) &incrementalEncoder};
	pdoEntryDatas[14] = {2, (void**) &incrementalEncoderTimestamp};
	pdoEntryDatas[15] = {2, (void**) &motorVoltage};
	pdoEntryDatas[16] = {2, (void**) &logicVoltage};
	pdoEntryDatas[17] = {2, (void**) &thermistor0};
	pdoEntryDatas[18] = {2, (void**) &thermistor1};
	pdoEntryDatas[19] = {2, (void**) &thermistor2};
	pdoEntryDatas[20] = {2, (void**) &thermistor3};
	pdoEntryDatas[21] = {2, (void**) &thermistor4};
	pdoEntryDatas[22] = {2, (void**) &thermistor5};
	pdoEntryDatas[23] = {2, (void**) &amp1MeasuredCurrent};
	pdoEntryDatas[24] = {2, (void**) &amp2MeasuredCurrent};
}

PDORegData LegMedulla::getPDORegData() {
	return {MEDULLA_LEG_RX_PDO_COUNT, MEDULLA_LEG_TX_PDO_COUNT,
	        pdoEntryDatas};
};

void LegMedulla::postOpInit() {
	motorEncoderValue                = (int64_t) *motorEncoder;
	motorEncoderTimestampValue       =           *motorEncoderTimestamp;
	legEncoderValue                  = (int64_t) *legEncoder;
	legEncoderTimestampValue         =           *legEncoderTimestamp;
	incrementalEncoderValue          =           *incrementalEncoder;
	incrementalEncoderTimestampValue =           *incrementalEncoderTimestamp;
	timingCounterValue               =           *timingCounter;
	updatePositionOffsets();
}

void LegMedulla::updatePositionOffsets() {
	skipMotorEncoder      = false;
	skipLegEncoder        = false;
	legPositionOffset     = 0.0;
    incrementalEncoderPos = 0.0;
	atrias_msgs::robot_state robotState;
	processPositions(robotState);
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			legPositionOffset = robotState.lLeg.halfA.motorAngle -
			                    robotState.lLeg.halfA.legAngle;
			incrementalEncoderStart = robotState.lLeg.halfA.motorAngle;
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			legPositionOffset = robotState.lLeg.halfB.motorAngle -
			                    robotState.lLeg.halfB.legAngle;
			incrementalEncoderStart = robotState.lLeg.halfB.motorAngle;
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			legPositionOffset = robotState.rLeg.halfA.motorAngle -
			                    robotState.rLeg.halfA.legAngle;
			incrementalEncoderStart = robotState.rLeg.halfA.motorAngle;
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			legPositionOffset = robotState.rLeg.halfB.motorAngle -
			                    robotState.rLeg.halfB.legAngle;
			incrementalEncoderStart = robotState.rLeg.halfB.motorAngle;
			break;
	}
	if (fabs(legPositionOffset) > MAX_LEG_POS_ADJUSTMENT) {
		log(RTT::Warning) << "Leg position adjustment limit exceeded! ID: "
		                  << getID() << RTT::endlog();
		legPositionOffset *= MAX_LEG_POS_ADJUSTMENT / fabs(legPositionOffset);
	}
}

void LegMedulla::checkErroneousEncoderValues() {
	skipMotorEncoder      = false;
	int32_t deltaMotorPos = *motorEncoder - motorEncoderValue;
	if (abs(deltaMotorPos) > MAX_ACCEPTABLE_ENCODER_CHANGE) {
		// Uncomment this if you want to debug issue 83
		/*log(RTT::Warning) << "Large motor encoder jump! Old value: "
		                  << motorEncoderValue << " New value: "
		                  << *motorEncoder << RTT::endlog();*/
		skipMotorEncoder = true;
	}
	
	skipLegEncoder      = false;
	int32_t deltaLegPos = *legEncoder - legEncoderValue;
	if (abs(deltaLegPos) > MAX_ACCEPTABLE_ENCODER_CHANGE)
		skipLegEncoder = true;
}

void LegMedulla::processIncrementalEncoders(RTT::os::TimeService::nsecs deltaTime, atrias_msgs::robot_state& robotState) {
	// This compensates for wraparound.
	int16_t deltaPos = ((int32_t) *incrementalEncoder + (1 << 15) - incrementalEncoderValue) % (1 << 16) - (1 << 15);
	incrementalEncoderValue += deltaPos;
	incrementalEncoderPos   += deltaPos;
	
	// Let's take into account the timestamps, too.
	double adjustedTime = ((double) deltaTime) / SECOND_IN_NANOSECONDS +
	                      ((double) (*incrementalEncoderTimestamp - incrementalEncoderTimestampValue))
	                      / MEDULLA_TIMER_FREQ;
	
	incrementalEncoderTimestampValue = *incrementalEncoderTimestamp;
	
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			robotState.lLeg.halfA.rotorAngle    =
				incrementalEncoderStart -
				INC_ENC_RAD_PER_TICK * incrementalEncoderPos * LEFT_MOTOR_A_DIRECTION;
			robotState.lLeg.halfA.rotorVelocity =
				((double) deltaPos) * INC_ENC_RAD_PER_TICK * -LEFT_MOTOR_A_DIRECTION / adjustedTime;
			break;
			
		case MEDULLA_LEFT_LEG_B_ID:
			robotState.lLeg.halfB.rotorAngle    =
				incrementalEncoderStart -
				INC_ENC_RAD_PER_TICK * incrementalEncoderPos * LEFT_MOTOR_B_DIRECTION;
			robotState.lLeg.halfB.rotorVelocity =
				((double) deltaPos) * INC_ENC_RAD_PER_TICK * -LEFT_MOTOR_B_DIRECTION / adjustedTime;
			break;
			
		case MEDULLA_RIGHT_LEG_A_ID:
			robotState.rLeg.halfA.rotorAngle    =
				incrementalEncoderStart -
				INC_ENC_RAD_PER_TICK * incrementalEncoderPos * RIGHT_MOTOR_A_DIRECTION;
			robotState.rLeg.halfA.rotorVelocity =
				((double) deltaPos) * INC_ENC_RAD_PER_TICK * -RIGHT_MOTOR_A_DIRECTION / adjustedTime;
			break;
			
		case MEDULLA_RIGHT_LEG_B_ID:
			robotState.rLeg.halfB.rotorAngle    =
				incrementalEncoderStart -
				INC_ENC_RAD_PER_TICK * incrementalEncoderPos * RIGHT_MOTOR_B_DIRECTION;
			robotState.rLeg.halfB.rotorVelocity =
				((double) deltaPos) * INC_ENC_RAD_PER_TICK * -RIGHT_MOTOR_B_DIRECTION / adjustedTime;
			break;
	}
}

void LegMedulla::processReceiveData(atrias_msgs::robot_state& robot_state) {
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
	
	checkErroneousEncoderValues();
	
	processPositions(robot_state);
	processVelocities(deltaTime, robot_state);
	processIncrementalEncoders(deltaTime, robot_state);
	processThermistors(robot_state);
	processLimitSwitches(robot_state, *state == medulla_state_idle);
	processVoltages(robot_state);
	processCurrents(robot_state);
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			robot_state.lLeg.halfA.medullaState = *state;
			robot_state.lLeg.halfA.errorFlags   = *errorFlags;
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			robot_state.lLeg.halfB.medullaState = *state;
			robot_state.lLeg.halfB.errorFlags   = *errorFlags;
			robot_state.lLeg.toeSwitch          = *toeSensor;
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			robot_state.rLeg.halfA.medullaState = *state;
			robot_state.rLeg.halfA.errorFlags   = *errorFlags;
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			robot_state.rLeg.halfB.medullaState = *state;
			robot_state.rLeg.halfB.errorFlags   = *errorFlags;
			robot_state.rLeg.toeSwitch          = *toeSensor;
			break;
	}
}

int32_t LegMedulla::calcMotorCurrentOut(atrias_msgs::controller_output& controllerOutput) {
	// Don't command any amount of torque if we're not enabled.
	if (controllerOutput.command != medulla_state_run) return 0;
	
	// If the ID isn't recognized, command 0 torque.
	double torqueCmd = 0.0;
	
	switch(*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			torqueCmd = controllerOutput.lLeg.motorCurrentA * LEFT_MOTOR_A_DIRECTION;
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			torqueCmd = controllerOutput.lLeg.motorCurrentB * LEFT_MOTOR_B_DIRECTION;
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			torqueCmd = controllerOutput.rLeg.motorCurrentA * RIGHT_MOTOR_A_DIRECTION;
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			torqueCmd = controllerOutput.rLeg.motorCurrentB * RIGHT_MOTOR_B_DIRECTION;
			break;
	}
	
	return (int32_t) (((double) MTR_MAX_COUNT) * torqueCmd / MTR_MAX_TORQUE);
}

inline double LegMedulla::encTicksToRad(uint32_t ticks, uint32_t calib_val, double rad_per_tick, double calib_val_rad) {
	// Be careful of integer/double conversions here.
	return calib_val_rad + ((double) (((int64_t) ticks) - ((int64_t) calib_val))) * rad_per_tick;
}

void LegMedulla::processPositions(atrias_msgs::robot_state& robotState) {
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			if (!skipMotorEncoder) {
				//log(RTT::Info) << "Left leg A motor encoder counts: " << *motorEncoder << RTT::endlog();
				robotState.lLeg.halfA.motorAngle =
					encTicksToRad(*motorEncoder, LEFT_TRAN_A_CALIB_VAL,  LEFT_TRAN_A_RAD_PER_CNT, LEG_A_CALIB_LOC);
			}
			if (!skipLegEncoder) {
				//log(RTT::Info) << "Left leg A leg encoder counts: " << *legEncoder << RTT::endlog();
				robotState.lLeg.halfA.legAngle   =
					encTicksToRad(*legEncoder,   LEFT_LEG_A_CALIB_VAL,   LEFT_LEG_A_RAD_PER_CNT,  LEG_A_CALIB_LOC) + legPositionOffset;
			}
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			if (!skipMotorEncoder) {
				//log(RTT::Info) << "Left leg B motor encoder counts: " << *motorEncoder << RTT::endlog();
				robotState.lLeg.halfB.motorAngle =
					encTicksToRad(*motorEncoder, LEFT_TRAN_B_CALIB_VAL,  LEFT_TRAN_B_RAD_PER_CNT, LEG_B_CALIB_LOC);
			}
			if (!skipLegEncoder) {
				//log(RTT::Info) << "Left leg B leg encoder counts: " << *legEncoder << RTT::endlog();
				robotState.lLeg.halfB.legAngle   =
					encTicksToRad(*legEncoder,   LEFT_LEG_B_CALIB_VAL,   LEFT_LEG_B_RAD_PER_CNT,  LEG_B_CALIB_LOC) + legPositionOffset;
			}
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			if (!skipMotorEncoder) {
				//log(RTT::Info) << "Right leg A motor encoder counts: " << *motorEncoder << RTT::endlog();
				robotState.rLeg.halfA.motorAngle =
					encTicksToRad(*motorEncoder, RIGHT_TRAN_A_CALIB_VAL, RIGHT_TRAN_A_RAD_PER_CNT, LEG_A_CALIB_LOC);
			}
			if (!skipLegEncoder) {
				//log(RTT::Info) << "Right leg A leg encoder counts: " << *legEncoder << RTT::endlog();
				robotState.rLeg.halfA.legAngle   =
					encTicksToRad(*legEncoder,   RIGHT_LEG_A_CALIB_VAL,  RIGHT_LEG_A_RAD_PER_CNT,  LEG_A_CALIB_LOC) + legPositionOffset;
			}
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			if (!skipMotorEncoder) {
				//log(RTT::Info) << "Right leg B motor encoder counts: " << *motorEncoder << RTT::endlog();
				robotState.rLeg.halfB.motorAngle =
					encTicksToRad(*motorEncoder, RIGHT_TRAN_B_CALIB_VAL, RIGHT_TRAN_B_RAD_PER_CNT, LEG_B_CALIB_LOC);
			}
			if (!skipLegEncoder) {
				//log(RTT::Info) << "Right leg B leg encoder counts: " << *legEncoder << RTT::endlog();
				robotState.rLeg.halfB.legAngle   =
					encTicksToRad(*legEncoder,   RIGHT_LEG_B_CALIB_VAL,  RIGHT_LEG_B_RAD_PER_CNT,  LEG_B_CALIB_LOC) + legPositionOffset;
			}
			break;
	}
}

void LegMedulla::processVelocities(RTT::os::TimeService::nsecs deltaTime, atrias_msgs::robot_state& robotState) {
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			if (!skipMotorEncoder) {
				// The division by 32 million translates timer ticks from the microcontroller into seconds
				robotState.lLeg.halfA.motorVelocity =
					((double) (((int64_t) *motorEncoder) - motorEncoderValue)) * LEFT_TRAN_A_RAD_PER_CNT  /
					(((double) deltaTime) / 1000000000.0 + ((double) (*motorEncoderTimestamp - motorEncoderTimestampValue)) / MEDULLA_TIMER_FREQ);
			}
			if (!skipLegEncoder) {
				robotState.lLeg.halfA.legVelocity   =
					((double) (((int64_t) *legEncoder)   - legEncoderValue))   * LEFT_LEG_A_RAD_PER_CNT   /
					(((double) deltaTime) / 1000000000.0 + ((double) (*legEncoderTimestamp   - legEncoderTimestampValue))   / MEDULLA_TIMER_FREQ);
			}
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			if (!skipMotorEncoder) {
				robotState.lLeg.halfB.motorVelocity =
					((double) (((int64_t) *motorEncoder) - motorEncoderValue)) * LEFT_TRAN_B_RAD_PER_CNT  /
					(((double) deltaTime) / 1000000000.0 + ((double) (*motorEncoderTimestamp - motorEncoderTimestampValue)) / MEDULLA_TIMER_FREQ);
			}
			if (!skipLegEncoder) {
				robotState.lLeg.halfB.legVelocity   =
					((double) (((int64_t) *legEncoder)   - legEncoderValue))   * LEFT_LEG_B_RAD_PER_CNT   /
					(((double) deltaTime) / 1000000000.0 + ((double) (*legEncoderTimestamp   -   legEncoderTimestampValue)) / MEDULLA_TIMER_FREQ);
			}
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			if (!skipMotorEncoder) {
				robotState.rLeg.halfA.motorVelocity =
					((double) (((int64_t) *motorEncoder) - motorEncoderValue)) * RIGHT_TRAN_A_RAD_PER_CNT /
					(((double) deltaTime) / 1000000000.0 + ((double) (*motorEncoderTimestamp - motorEncoderTimestampValue)) / MEDULLA_TIMER_FREQ);
			}
			if (!skipLegEncoder) {
				robotState.rLeg.halfA.legVelocity   =
					((double) (((int64_t) *legEncoder)   - legEncoderValue))   * RIGHT_LEG_A_RAD_PER_CNT  /
					(((double) deltaTime) / 1000000000.0 + ((double) (*legEncoderTimestamp   - legEncoderTimestampValue))   / MEDULLA_TIMER_FREQ);
			}
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			if (!skipMotorEncoder) {
				robotState.rLeg.halfB.motorVelocity =
					((double) (((int64_t) *motorEncoder) - motorEncoderValue)) * RIGHT_TRAN_B_RAD_PER_CNT /
					(((double) deltaTime) / 1000000000.0 + ((double) (*motorEncoderTimestamp - motorEncoderTimestampValue)) / MEDULLA_TIMER_FREQ);
			}
			if (!skipLegEncoder) {
				robotState.rLeg.halfB.legVelocity   =
					((double) (((int64_t) *legEncoder)   - legEncoderValue))   * RIGHT_LEG_B_RAD_PER_CNT  /
					(((double) deltaTime) / 1000000000.0 + ((double) (*legEncoderTimestamp   -   legEncoderTimestampValue)) / MEDULLA_TIMER_FREQ);
			}
			break;
	}

	if (!skipMotorEncoder) {
		motorEncoderValue          = (int64_t) *motorEncoder;
		motorEncoderTimestampValue = *motorEncoderTimestamp;
	} else {
		motorEncoderTimestampValue -= deltaTime * 32000; // Number of timer ticks in a millisecond.
	}
	if (!skipLegEncoder) {
		legEncoderValue          = (int64_t) *legEncoder;
		legEncoderTimestampValue = *legEncoderTimestamp;
	} else {
		legEncoderTimestampValue -= deltaTime * 32000;
	}
}

void LegMedulla::processThermistors(atrias_msgs::robot_state& robotState) {
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			robotState.lLeg.halfA.motorTherms[0] = processThermistorValue(*thermistor0);
			robotState.lLeg.halfA.motorTherms[1] = processThermistorValue(*thermistor1);
			robotState.lLeg.halfA.motorTherms[2] = processThermistorValue(*thermistor2);
			robotState.lLeg.halfA.motorTherms[3] = processThermistorValue(*thermistor3);
			robotState.lLeg.halfA.motorTherms[4] = processThermistorValue(*thermistor4);
			robotState.lLeg.halfA.motorTherms[5] = processThermistorValue(*thermistor5);
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			robotState.lLeg.halfB.motorTherms[0] = processThermistorValue(*thermistor0);
			robotState.lLeg.halfB.motorTherms[1] = processThermistorValue(*thermistor1);
			robotState.lLeg.halfB.motorTherms[2] = processThermistorValue(*thermistor2);
			robotState.lLeg.halfB.motorTherms[3] = processThermistorValue(*thermistor3);
			robotState.lLeg.halfB.motorTherms[4] = processThermistorValue(*thermistor4);
			robotState.lLeg.halfB.motorTherms[5] = processThermistorValue(*thermistor5);
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			robotState.rLeg.halfA.motorTherms[0] = processThermistorValue(*thermistor0);
			robotState.rLeg.halfA.motorTherms[1] = processThermistorValue(*thermistor1);
			robotState.rLeg.halfA.motorTherms[2] = processThermistorValue(*thermistor2);
			robotState.rLeg.halfA.motorTherms[3] = processThermistorValue(*thermistor3);
			robotState.rLeg.halfA.motorTherms[4] = processThermistorValue(*thermistor4);
			robotState.rLeg.halfA.motorTherms[5] = processThermistorValue(*thermistor5);
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			robotState.rLeg.halfB.motorTherms[0] = processThermistorValue(*thermistor0);
			robotState.rLeg.halfB.motorTherms[1] = processThermistorValue(*thermistor1);
			robotState.rLeg.halfB.motorTherms[2] = processThermistorValue(*thermistor2);
			robotState.rLeg.halfB.motorTherms[3] = processThermistorValue(*thermistor3);
			robotState.rLeg.halfB.motorTherms[4] = processThermistorValue(*thermistor4);
			robotState.rLeg.halfB.motorTherms[5] = processThermistorValue(*thermistor5);
			break;
	}
}

void LegMedulla::processCurrents(atrias_msgs::robot_state& robotState) {
	double current1 = processAmplifierCurrent(*amp1MeasuredCurrent);
	double current2 = processAmplifierCurrent(*amp2MeasuredCurrent);
	
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			robotState.lLeg.halfA.amp1Current  = current1;
			robotState.lLeg.halfA.amp2Current  = current2;
			robotState.lLeg.halfA.motorCurrent = current1 + current2;
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			robotState.lLeg.halfB.amp1Current  = current1;
			robotState.lLeg.halfB.amp2Current  = current2;
			robotState.lLeg.halfB.motorCurrent = current1 + current2;
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			robotState.rLeg.halfA.amp1Current  = current1;
			robotState.rLeg.halfA.amp2Current  = current2;
			robotState.rLeg.halfA.motorCurrent = current1 + current2;
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			robotState.rLeg.halfB.amp1Current  = current1;
			robotState.rLeg.halfB.amp2Current  = current2;
			robotState.rLeg.halfB.motorCurrent = current1 + current2;
			break;
	}
}

void LegMedulla::processLimitSwitches(atrias_msgs::robot_state& robotState, bool reset) {
	atrias_msgs::robot_state_leg* leg;
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			// Fallthrough
		case MEDULLA_LEFT_LEG_B_ID:
			leg = &(robotState.lLeg);
			break;
			
		case MEDULLA_RIGHT_LEG_A_ID:
			// Fallthrough
		case MEDULLA_RIGHT_LEG_B_ID:
			leg = &(robotState.rLeg);
			break;
			
		default:
			return;
	}

	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			// Fallthrough
		case MEDULLA_RIGHT_LEG_A_ID:
			if (reset)
				leg->halfA.limitSwitches = 0;
			leg->halfA.limitSwitches |= *limitSwitch;
			leg->halfA.motorNegLimitSwitch = (leg->halfA.limitSwitches) & (1 << 0);
			leg->halfA.motorPosLimitSwitch = (leg->halfA.limitSwitches) & (1 << 1);
			leg->halfA.negDeflectSwitch    = (leg->halfA.limitSwitches) & (1 << 2);
			leg->halfA.posDeflectSwitch    = (leg->halfA.limitSwitches) & (1 << 3);
			leg->legExtendSwitch           = (leg->halfA.limitSwitches) & (1 << 4);
			leg->legRetractSwitch          = (leg->halfA.limitSwitches) & (1 << 5);
			break;
			
		case MEDULLA_LEFT_LEG_B_ID:
			// Fallthrough
		case MEDULLA_RIGHT_LEG_B_ID:
			if (reset)
				leg->halfB.limitSwitches = 0;
			leg->halfB.limitSwitches |= *limitSwitch;
			leg->halfB.motorNegLimitSwitch = (leg->halfB.limitSwitches) & (1 << 0);
			leg->halfB.motorPosLimitSwitch = (leg->halfB.limitSwitches) & (1 << 1);
			leg->halfB.negDeflectSwitch    = (leg->halfB.limitSwitches) & (1 << 2);
			leg->halfB.posDeflectSwitch    = (leg->halfB.limitSwitches) & (1 << 3);
			leg->motorRetractSwitch        = (leg->halfB.limitSwitches) & (1 << 4);
			break;
	}
}

void LegMedulla::processVoltages(atrias_msgs::robot_state& robotState) {
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			robotState.lLeg.halfA.motorVoltage = processADCValue(*motorVoltage) * 30.0;
			robotState.lLeg.halfA.logicVoltage = processADCValue(*logicVoltage) *  6.0;
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			robotState.lLeg.halfB.motorVoltage = processADCValue(*motorVoltage) * 30.0;
			robotState.lLeg.halfB.logicVoltage = processADCValue(*logicVoltage) *  6.0;
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			robotState.rLeg.halfA.motorVoltage = processADCValue(*motorVoltage) * 30.0;
			robotState.rLeg.halfA.logicVoltage = processADCValue(*logicVoltage) *  6.0;
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			robotState.rLeg.halfB.motorVoltage = processADCValue(*motorVoltage) * 30.0;
			robotState.rLeg.halfB.logicVoltage = processADCValue(*logicVoltage) *  6.0;
			break;
	}
}

void LegMedulla::processTransmitData(atrias_msgs::controller_output& controller_output) {
	*counter      = ++local_counter;
	*command      = controller_output.command;
	*motorCurrent = calcMotorCurrentOut(controller_output);
}

uint8_t LegMedulla::getID() {
	return *id;
}

}

}
