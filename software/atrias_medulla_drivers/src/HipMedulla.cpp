#include "atrias_medulla_drivers/HipMedulla.h"

namespace atrias {

namespace medullaDrivers {

HipMedulla::HipMedulla() : Medulla() {
	pdoEntryDatas[0]  = {1, (void**) &command};
	pdoEntryDatas[1]  = {2, (void**) &counter};
	pdoEntryDatas[2]  = {4, (void**) &motorCurrent};
	pdoEntryDatas[3]  = {1, (void**) &id};
	pdoEntryDatas[4]  = {1, (void**) &state};
	pdoEntryDatas[5]  = {1, (void**) &timingCounter};
	pdoEntryDatas[6]  = {1, (void**) &errorFlags};
	pdoEntryDatas[7]  = {1, (void**) &limitSwitches};
	pdoEntryDatas[8]  = {4, (void**) &hipEncoder};
	pdoEntryDatas[9]  = {2, (void**) &hipEncoderTimestamp};
	pdoEntryDatas[10] = {2, (void**) &motorVoltage};
	pdoEntryDatas[11] = {2, (void**) &logicVoltage};
	pdoEntryDatas[12] = {2, (void**) &thermistor0};
	pdoEntryDatas[13] = {2, (void**) &thermistor1};
	pdoEntryDatas[14] = {2, (void**) &thermistor2};
	pdoEntryDatas[15] = {2, (void**) &ampMeasuredCurrent};
	pdoEntryDatas[16] = {4, (void**) &accelX};
	pdoEntryDatas[17] = {4, (void**) &accelY};
	pdoEntryDatas[18] = {4, (void**) &accelZ};
	pdoEntryDatas[19] = {4, (void**) &angRateX};
	pdoEntryDatas[20] = {4, (void**) &angRateY};
	pdoEntryDatas[21] = {4, (void**) &angRateZ};
	pdoEntryDatas[22] = {4, (void**) &m11};
	pdoEntryDatas[23] = {4, (void**) &m12};
	pdoEntryDatas[24] = {4, (void**) &m13};
	pdoEntryDatas[25] = {4, (void**) &m21};
	pdoEntryDatas[26] = {4, (void**) &m22};
	pdoEntryDatas[27] = {4, (void**) &m23};
	pdoEntryDatas[28] = {4, (void**) &m31};
	pdoEntryDatas[29] = {4, (void**) &m32};
	pdoEntryDatas[30] = {4, (void**) &m33};
	pdoEntryDatas[31] = {4, (void**) &timer};
	pdoEntryDatas[32] = {2, (void**) &incrementalEncoder};
	pdoEntryDatas[33] = {2, (void**) &incrementalEncoderTimestamp};
}

PDORegData HipMedulla::getPDORegData() {
	return {MEDULLA_HIP_RX_PDO_COUNT, MEDULLA_HIP_TX_PDO_COUNT,
	        pdoEntryDatas};
}

void HipMedulla::postOpInit() {
printf("%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n%p\n",
command,
counter,
motorCurrent,
id,
state,
timingCounter,
errorFlags,
limitSwitches,
hipEncoder,
hipEncoderTimestamp,
motorVoltage,
logicVoltage,
thermistor0,
thermistor1,
thermistor2,
ampMeasuredCurrent,
accelX,
accelY,
accelZ,
angRateX,
angRateY,
angRateZ,
m11,
m12,
m13,
m21,
m22,
m23,
m31,
m32,
m33,
timer,
incrementalEncoder,
incrementalEncoderTimestamp);
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

void HipMedulla::updateLimitSwitches(atrias_msgs::robot_state_hip& hip, bool reset) {
	if (reset)
		hip.limitSwitches = 0;
	hip.limitSwitches     |= *limitSwitches;
	hip.InsideLimitSwitch  = hip.limitSwitches & (1 << 0);
	hip.OutsideLimitSwitch = hip.limitSwitches & (1 << 1);
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
	
	updateLimitSwitches(hip, *state == medulla_state_idle);
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
