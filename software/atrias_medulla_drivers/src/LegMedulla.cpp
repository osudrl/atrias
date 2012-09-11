#include "atrias_medulla_drivers/LegMedulla.h"

namespace atrias {

namespace medullaDrivers {

LegMedulla::LegMedulla(intptr_t outputs[], intptr_t inputs[]) :
            Medulla() {
	
	command      = (uint8_t*)  (outputs[0]);
	counter      = (uint16_t*) (outputs[1]);
	motorCurrent = (int32_t*)  (outputs[2]);

	id                          = (uint8_t*)  (inputs[0]);
	state                       = (uint8_t*)  (inputs[1]);
	timingCounter               = (uint8_t*)  (inputs[2]);
	errorFlags                  = (uint8_t*)  (inputs[3]);
	limitSwitch                 = (uint8_t*)  (inputs[4]);
	toeSensor                   = (uint16_t*) (inputs[5]);
	motorEncoder                = (uint32_t*) (inputs[6]);
	motorEncoderTimestamp       = (int16_t*)  (inputs[7]);
	incrementalEncoder          = (uint16_t*) (inputs[8]);
	incrementalEncoderTimestamp = (uint16_t*) (inputs[9]);
	legEncoder                  = (uint32_t*) (inputs[10]);
	legEncoderTimestamp         = (int16_t*)  (inputs[11]);
	motorVoltage                = (uint16_t*) (inputs[12]);
	logicVoltage                = (uint16_t*) (inputs[13]);
	thermistor0                 = (uint16_t*) (inputs[14]);
	thermistor1                 = (uint16_t*) (inputs[15]);
	thermistor2                 = (uint16_t*) (inputs[16]);
	thermistor3                 = (uint16_t*) (inputs[17]);
	thermistor4                 = (uint16_t*) (inputs[18]);
	thermistor5                 = (uint16_t*) (inputs[19]);
	amp1MeasuredCurrent         = (int16_t*)  (inputs[20]);
	amp2MeasuredCurrent         = (int16_t*)  (inputs[21]);
	
	motorEncoderValue                = (int64_t) *motorEncoder;
	motorEncoderTimestampValue       =           *motorEncoderTimestamp;
	legEncoderValue                  = (int64_t) *legEncoder;
	legEncoderTimestampValue         =           *legEncoderTimestamp;
	incrementalEncoderValue          =           *incrementalEncoder;
	incrementalEncoderTimestampValue =           *incrementalEncoderTimestamp;
	timingCounterValue               =           *timingCounter;
}

intptr_t LegMedulla::getInputsSize() {
	return MEDULLA_LEG_INPUTS_SIZE;
}

intptr_t LegMedulla::getOutputsSize() {
	return MEDULLA_LEG_OUTPUTS_SIZE;
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
	
	// Let's take into account the timestamps, too.
	double adjustedTime = ((double) deltaTime) / SECOND_IN_NANOSECONDS +
	                      ((double) (*incrementalEncoderTimestamp - incrementalEncoderTimestampValue))
	                      / MEDULLA_TIMER_FREQ;
	
	incrementalEncoderTimestampValue = *incrementalEncoderTimestamp;
	
	switch (*id) {
		case MEDULLA_LEFT_LEG_A_ID:
			robotState.lLeg.halfA.rotorAngle += deltaPos * INC_ENC_RAD_PER_TICK;
			robotState.lLeg.halfA.rotorVelocity =
				((double) deltaPos) * INC_ENC_RAD_PER_TICK / adjustedTime;
			break;
			
		case MEDULLA_LEFT_LEG_B_ID:
			robotState.lLeg.halfB.rotorAngle += deltaPos * INC_ENC_RAD_PER_TICK;
			robotState.lLeg.halfB.rotorVelocity =
				((double) deltaPos) * INC_ENC_RAD_PER_TICK / adjustedTime;
			break;
			
		case MEDULLA_RIGHT_LEG_A_ID:
			robotState.rLeg.halfA.rotorAngle += deltaPos * INC_ENC_RAD_PER_TICK;
			robotState.rLeg.halfA.rotorVelocity =
				((double) deltaPos) * INC_ENC_RAD_PER_TICK / adjustedTime;
			break;
			
		case MEDULLA_RIGHT_LEG_B_ID:
			robotState.rLeg.halfB.rotorAngle += deltaPos * INC_ENC_RAD_PER_TICK;
			robotState.rLeg.halfB.rotorVelocity =
				((double) deltaPos) * INC_ENC_RAD_PER_TICK / adjustedTime;
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
	processLimitSwitches(robot_state);
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
				robotState.lLeg.halfA.motorAngle =
					encTicksToRad(*motorEncoder, LEFT_TRAN_A_CALIB_VAL,  LEFT_TRAN_A_RAD_PER_CNT, LEG_A_CALIB_LOC);
			}
			if (!skipLegEncoder) {
				robotState.lLeg.halfA.legAngle   =
					encTicksToRad(*legEncoder,   LEFT_LEG_A_CALIB_VAL,   LEFT_LEG_A_RAD_PER_CNT,  LEG_A_CALIB_LOC);
			}
			break;
		case MEDULLA_LEFT_LEG_B_ID:
			if (!skipMotorEncoder) {
				robotState.lLeg.halfB.motorAngle =
					encTicksToRad(*motorEncoder, LEFT_TRAN_B_CALIB_VAL,  LEFT_TRAN_B_RAD_PER_CNT, LEG_B_CALIB_LOC);
			}
			if (!skipLegEncoder) {
				robotState.lLeg.halfB.legAngle   =
					encTicksToRad(*legEncoder,   LEFT_LEG_B_CALIB_VAL,   LEFT_LEG_B_RAD_PER_CNT,  LEG_B_CALIB_LOC);
			}
			break;
		case MEDULLA_RIGHT_LEG_A_ID:
			if (!skipMotorEncoder) {
				robotState.rLeg.halfA.motorAngle =
					encTicksToRad(*motorEncoder, RIGHT_TRAN_A_CALIB_VAL, RIGHT_TRAN_A_RAD_PER_CNT, LEG_A_CALIB_LOC);
			}
			if (!skipLegEncoder) {
				robotState.rLeg.halfA.legAngle   =
					encTicksToRad(*legEncoder,   RIGHT_LEG_A_CALIB_VAL,  RIGHT_LEG_A_RAD_PER_CNT,  LEG_A_CALIB_LOC);
			}
			break;
		case MEDULLA_RIGHT_LEG_B_ID:
			if (!skipMotorEncoder) {
				robotState.rLeg.halfB.motorAngle =
					encTicksToRad(*motorEncoder, RIGHT_TRAN_B_CALIB_VAL, RIGHT_TRAN_B_RAD_PER_CNT, LEG_B_CALIB_LOC);
			}
			if (!skipLegEncoder) {
				robotState.rLeg.halfB.legAngle   =
					encTicksToRad(*legEncoder,   RIGHT_LEG_B_CALIB_VAL,  RIGHT_LEG_B_RAD_PER_CNT,  LEG_B_CALIB_LOC);
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
					((double) (((int64_t) *legEncoder)   - legEncoderValue))   * RIGHT_LEG_B_RAD_PER_CNT  /
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

void LegMedulla::processLimitSwitches(atrias_msgs::robot_state& robotState) {
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
			leg->halfA.motorNegLimitSwitch = (*limitSwitch) & (1 << 0);
			leg->halfA.motorPosLimitSwitch = (*limitSwitch) & (1 << 1);
			leg->halfA.negDeflectSwitch    = (*limitSwitch) & (1 << 2);
			leg->halfA.posDeflectSwitch    = (*limitSwitch) & (1 << 3);
			leg->legExtendSwitch           = (*limitSwitch) & (1 << 4);
			leg->legRetractSwitch          = (*limitSwitch) & (1 << 5);
			break;
			
		case MEDULLA_LEFT_LEG_B_ID:
			// Fallthrough
		case MEDULLA_RIGHT_LEG_B_ID:
			leg->halfB.motorNegLimitSwitch = (*limitSwitch) & (1 << 0);
			leg->halfB.motorPosLimitSwitch = (*limitSwitch) & (1 << 1);
			leg->halfB.negDeflectSwitch    = (*limitSwitch) & (1 << 2);
			leg->halfB.posDeflectSwitch    = (*limitSwitch) & (1 << 3);
			leg->motorRetractSwitch        = (*limitSwitch) & (1 << 4);
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
