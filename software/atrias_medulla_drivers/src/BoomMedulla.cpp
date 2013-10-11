#include "atrias_medulla_drivers/BoomMedulla.h"

namespace atrias {
namespace medullaDrivers {

BoomMedulla::BoomMedulla() : Medulla() {
	pdoEntryDatas[0] = {1, (void**) &command};
	pdoEntryDatas[1] = {2, (void**) &counter};
	pdoEntryDatas[2] = {1, (void**) &id};
	pdoEntryDatas[3] = {1, (void**) &state};
	pdoEntryDatas[4] = {1, (void**) &timingCounter};
	pdoEntryDatas[5] = {1, (void**) &errorFlags};
	pdoEntryDatas[6] = {4, (void**) &xEncoder};
	pdoEntryDatas[7] = {2, (void**) &xTimestamp};
	pdoEntryDatas[8] = {4, (void**) &pitchEncoder};
	pdoEntryDatas[9] = {2, (void**) &pitchTimestamp};
	pdoEntryDatas[10] = {4, (void**) &zEncoder};
	pdoEntryDatas[11] = {2, (void**) &zTimestamp};
	pdoEntryDatas[12] = {2, (void**) &logicVoltage};
}

PDORegData BoomMedulla::getPDORegData() {
	return {MEDULLA_BOOM_RX_PDO_COUNT, MEDULLA_BOOM_TX_PDO_COUNT, pdoEntryDatas};
};

void BoomMedulla::postOpInit() {
	// X position encoder
	xEncoderDecoder.init(BOOM_ENCODER_BITS, *xEncoder, 0.0, BOOM_X_METERS_PER_TICK);
	
	// X angle encoder
	xAngleDecoder.init(BOOM_ENCODER_BITS, *xEncoder, 0.0, -2.0 * M_PI / (1 << BOOM_ENCODER_BITS) / BOOM_X_GEAR_RATIO);

	// Body pitch encoder
	pitchEncoderPos = 
		(*pitchEncoder - BOOM_PITCH_VERTICAL_VALUE) % (1 << BOOM_ENCODER_BITS);
	
	// Compensate for the difference between % and modulo
	pitchEncoderPos += 1 << BOOM_ENCODER_BITS;
	
	// Compensate for wraparound
	pitchEncoderPos = 
		(pitchEncoderPos + (1 << (BOOM_ENCODER_BITS - 1))) % 
		(1 << BOOM_ENCODER_BITS) - (1 << (BOOM_ENCODER_BITS - 1));
	
	pitchEncoderValue = *pitchEncoder;
	pitchTimestampValue = *pitchTimestamp;
	
	// Z Position encoder
	zEncoderPos = 
		(*zEncoder - BOOM_Z_CALIB_VAL) % (1 << BOOM_ENCODER_BITS);
	
	// Compensate for the difference between % and modulo
	zEncoderPos += 1 << BOOM_ENCODER_BITS;
	
	// Compensate for wraparound
	zEncoderPos = 
		(zEncoderPos + (1 << (BOOM_ENCODER_BITS - 1))) % 
		(1 << BOOM_ENCODER_BITS) - (1 << (BOOM_ENCODER_BITS - 1));
	
	zEncoderValue = *zEncoder;
	zTimestampValue = *zTimestamp;
}

uint8_t BoomMedulla::getID() {
	return *id;
}

void BoomMedulla::processXEncoder(RTT::os::TimeService::nsecs deltaTime, atrias_msgs::robot_state& robotState) {
	// X position encoder (robot)
	xEncoderDecoder.update(*xEncoder, deltaTime, *xTimestamp);
	robotState.position.xPosition = xEncoderDecoder.getPos();
	robotState.position.xVelocity = xEncoderDecoder.getVel();

	// X angle encoder (boom yaw)
	xAngleDecoder.update(*xEncoder, deltaTime, *xTimestamp);
	robotState.position.xAngle = xAngleDecoder.getPos();
	robotState.position.xAngleVelocity = xAngleDecoder.getVel();
}

void BoomMedulla::processPitchEncoder(RTT::os::TimeService::nsecs deltaTime, atrias_msgs::robot_state& robotState) {
     // DEBUG statements for calibration
	//log(RTT::Info) << "Boom pitch encoder counts: " << *pitchEncoder << RTT::endlog();
	
	// Obtain the deltas
	int deltaPos = ((int32_t) *pitchEncoder) - ((int32_t) pitchEncoderValue);
	
	// Compensate for the difference between the % operator and the modulo operation
	deltaPos += 1 << BOOM_ENCODER_BITS;
	
	// Compensate for rollover
	deltaPos = 
		(deltaPos + (1 << (BOOM_ENCODER_BITS - 1))) % 
		(1 << BOOM_ENCODER_BITS) - (1 << (BOOM_ENCODER_BITS - 1));
	
	pitchEncoderPos += deltaPos;
	
	robotState.position.bodyPitch = 
		pitchEncoderPos * PITCH_ENCODER_RAD_PER_TICK + 3.0 * M_PI / 2.0;
	
	robotState.position.bodyPitchVelocity =
		deltaPos * PITCH_ENCODER_RAD_PER_TICK /
		(((double) deltaTime) / ((double) SECOND_IN_NANOSECONDS) + 
		((double) (((int16_t) *pitchTimestamp) - pitchTimestampValue)) /
		MEDULLA_TIMER_FREQ);
	
	pitchEncoderValue = *pitchEncoder;
	pitchTimestampValue = *pitchTimestamp;
	robotState.position.pitchEncoderRaw = pitchEncoderValue;
}

void BoomMedulla::processZEncoder(RTT::os::TimeService::nsecs deltaTime, atrias_msgs::robot_state&   robotState) {
	// DEBUG statements for calibration
	//log(RTT::Info) << "Z Encoder value: " << *zEncoder << RTT::endlog();
	
	// Obtain the deltas
	int deltaPos = ((int32_t) *zEncoder) - ((int32_t) zEncoderValue);
	double actualDeltaTime =
		((double) deltaTime) / ((double) SECOND_IN_NANOSECONDS) +
		((double) (((int16_t) *zTimestamp) - zTimestampValue)) /
		((double) MEDULLA_TIMER_FREQ);
	
	// Compensate for the difference between the % operator and the modulo operation
	deltaPos += 1 << BOOM_ENCODER_BITS;
	
	// Compensate for rollover
	deltaPos = 
		(deltaPos + (1 << (BOOM_ENCODER_BITS - 1))) % 
		(1 << BOOM_ENCODER_BITS) - (1 << (BOOM_ENCODER_BITS - 1));
	zEncoderPos += deltaPos;
	
	// Compute the boom angle (boom pitch)
	robotState.position.boomAngle = 
		zEncoderPos * BOOM_Z_ENCODER_RAD_PER_TICK + BOOM_Z_CALIB_LOC;
	
	// The angle of the line between the boom's pivot and the robot's origin
	double virtualBoomAngle = 
		robotState.position.boomAngle + atan2(BOOM_ROBOT_VERTICAL_OFFSET, BOOM_LENGTH);

	robotState.position.yPosition = 
		-cos(virtualBoomAngle) * BOOM_LENGTH;
	robotState.position.zPosition = 
		BOOM_HEIGHT + BOOM_LENGTH * sin(virtualBoomAngle);

	robotState.position.yVelocity = 
		BOOM_LENGTH * robotState.position.boomAngleVelocity * sin(virtualBoomAngle);
	robotState.position.zVelocity =
		BOOM_LENGTH * robotState.position.boomAngleVelocity * cos(virtualBoomAngle);
	
	// Compute robot x position (defined at the center of the hip-torso pivot axis)
	//robotState.position.xPosition = BOOM_LENGTH * cos(robotState.position.boomAngle) * cos(robotState.position.xAngle) - TORSO_LENGTH * (sin(BOOM_TORSO_OFFSET) * (sin(robotState.position.xAngle) * sin(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) + cos(robotState.position.xAngle) * cos(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * sin(robotState.position.boomAngle)) - cos(robotState.position.boomAngle) * cos(robotState.position.xAngle) * cos(BOOM_TORSO_OFFSET));

	// Compute robot y position (defined at the center of the hip-torso pivot axis)
	//robotState.position.yPosition = TORSO_LENGTH * (sin(BOOM_TORSO_OFFSET) * (cos(robotState.position.xAngle) * sin(robotState.position.bodyPitch - 3.0 * M_PI /2.0) - cos(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * sin(robotState.position.boomAngle) * sin(robotState.position.xAngle)) + cos(robotState.position.boomAngle) * sin(robotState.position.xAngle) * cos(BOOM_TORSO_OFFSET)) + BOOM_LENGTH * cos(robotState.position.boomAngle) * sin(robotState.position.xAngle);
	
	// Compute robot z position (defined at the center of the hip-torso pivot axis)
	robotState.position.zPosition = BOOM_HEIGHT + BOOM_LENGTH * sin(robotState.position.boomAngle) + TORSO_LENGTH * (sin(robotState.position.boomAngle) * cos(BOOM_TORSO_OFFSET) + cos(robotState.position.boomAngle) * cos(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * sin(BOOM_TORSO_OFFSET));
	
	// Compute robot position (the arc length of radial trajectory around boom)
	// 

	// Compute the boom angle velocity (boom pitch velocity)
	robotState.position.boomAngleVelocity =
		((double) deltaPos) * BOOM_Z_ENCODER_RAD_PER_TICK / actualDeltaTime;
	
	// Compute robot x velocity (defined at the center of the hip-torso pivot axis)
	//robotState.position.xVelocity = - TORSO_LENGTH * (sin(BOOM_TORSO_OFFSET) * (cos(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * sin(robotState.position.xAngle) * robotState.position.bodyPitchVelocity + cos(robotState.position.xAngle) * sin(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * robotState.position.xAngleVelocity + cos(robotState.position.boomAngle) * cos(robotState.position.xAngle) * cos(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * robotState.position.boomAngleVelocity - cos(robotState.position.xAngle) * sin(robotState.position.boomAngle) * sin(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * robotState.position.bodyPitchVelocity - cos(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * sin(robotState.position.boomAngle) * sin(robotState.position.xAngle) * robotState.position.xAngleVelocity) + cos(robotState.position.xAngle) * sin(robotState.position.boomAngle) * cos(BOOM_TORSO_OFFSET) * robotState.position.boomAngleVelocity + cos(robotState.position.boomAngle) * sin(robotState.position.xAngle) * cos(BOOM_TORSO_OFFSET) * robotState.position.xAngleVelocity) - BOOM_LENGTH * cos(robotState.position.xAngle) * sin(robotState.position.boomAngle) * robotState.position.boomAngleVelocity - BOOM_LENGTH * cos(robotState.position.boomAngle) * sin(robotState.position.xAngle) * robotState.position.xAngleVelocity;

	// Compute robot y velocity (defined at the center of the hip-torso pivot axis)
	//robotState.position.yVelocity = BOOM_LENGTH * cos(robotState.position.boomAngle) * cos(robotState.position.xAngle) * robotState.position.xAngleVelocity - BOOM_LENGTH * sin(robotState.position.boomAngle) * sin(robotState.position.xAngle) * robotState.position.boomAngleVelocity - TORSO_LENGTH * (sin(BOOM_TORSO_OFFSET) * (sin(robotState.position.xAngle) * sin(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * robotState.position.xAngleVelocity - cos(robotState.position.xAngle) * cos(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * robotState.position.bodyPitchVelocity + cos(robotState.position.boomAngle) * cos(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * sin(robotState.position.xAngle) * robotState.position.boomAngleVelocity + cos(robotState.position.xAngle) * cos(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * sin(robotState.position.boomAngle) * robotState.position.xAngleVelocity - sin(robotState.position.boomAngle) * sin(robotState.position.xAngle) * sin(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * robotState.position.bodyPitchVelocity) - cos(robotState.position.boomAngle) * cos(robotState.position.xAngle) * cos(BOOM_TORSO_OFFSET) * robotState.position.xAngleVelocity + sin(robotState.position.boomAngle) * sin(robotState.position.xAngle) * cos(BOOM_TORSO_OFFSET) * robotState.position.boomAngleVelocity);

	// Compute robot z velocity (defined at the center of the hip-torso pivot axis)
	robotState.position.zVelocity = BOOM_LENGTH * cos(robotState.position.boomAngle) * robotState.position.boomAngleVelocity - TORSO_LENGTH * (cos(robotState.position.boomAngle) * sin(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * sin(BOOM_TORSO_OFFSET) * robotState.position.bodyPitchVelocity - cos(robotState.position.boomAngle) * cos(BOOM_TORSO_OFFSET) * robotState.position.boomAngleVelocity + cos(robotState.position.bodyPitch - 3.0 * M_PI / 2.0) * sin(robotState.position.boomAngle) * sin(BOOM_TORSO_OFFSET) * robotState.position.boomAngleVelocity);
	
	// Compute robot velocity (the tangential velocity around boom)
	//

	zEncoderValue = *zEncoder;
	zTimestampValue = *zTimestamp;

	robotState.position.zEncoderRaw = *zEncoder;
}

void BoomMedulla::processTransmitData(atrias_msgs::controller_output& controller_output) {
	*counter = ++local_counter;
	*command = controller_output.command;
}

void BoomMedulla::processReceiveData(atrias_msgs::robot_state& robot_state) {
	// If we don't have new data, don't run. It's pointless, and results in
	// NaN velocities.
	if (*timingCounter == timingCounterValue)
		return;

	// Calculate how much time has elapsed since the previous sensor readings.
	// Note: % isn't actually a modulo, hence the additional 256.
	RTT::os::TimeService::nsecs deltaTime =
		((((int16_t) *timingCounter) + 256 - ((int16_t) timingCounterValue)) % 256) *
		CONTROLLER_LOOP_PERIOD_NS;
	timingCounterValue = *timingCounter;
	
	processPitchEncoder(deltaTime, robot_state);
	processXEncoder(deltaTime, robot_state);
	processZEncoder(deltaTime, robot_state);
	
	robot_state.boomMedullaState = *state;
	robot_state.boomMedullaErrorFlags = *errorFlags;
	robot_state.boomLogicVoltage = decodeLogicVoltage(*logicVoltage);
}

} // medullaDrivers
} // atrias

// vim: noexpandtab
