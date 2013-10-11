#include "atrias_rt_ops/Safety.h"

namespace atrias {

namespace rtOps {

Safety::Safety(RTOps* rt_ops) {
	rtOps     = rt_ops;
	isHalting = false;
}

bool Safety::motorHaltCheck(double vel, double &minVel, double &maxVel) {
	// The rate at which we should be decelerating (rad/s/tick)
	double decelRate = AVAIL_HALT_AMPS * ACCEL_PER_AMP * CONTROLLER_LOOP_PERIOD_NS / SECOND_IN_NANOSECONDS;

	// Update velocity interval
	minVel = std::min(minVel + decelRate, -MOTOR_VEL_MRGN);
	maxVel = std::max(maxVel - decelRate,  MOTOR_VEL_MRGN);

	// If we're not in the acceptable interval, return true.
	if (vel < minVel)
		return true;
	if (vel > maxVel)
		return true;

	// We're at an acceptable velocity -- return false
	return false;
}

double Safety::predictStop(double pos, double vel) {
	// Derived from basic physics -- assumes constant stopping acceleration.
	return pos + vel * abs(vel) / (2.0 * ACCEL_PER_AMP * AVAIL_HALT_AMPS);
}

bool Safety::shouldEStop(atrias_msgs::controller_output &co) {
	atrias_msgs::robot_state robotState = rtOps->getRobotStateHandler()->getRobotState();

	// Check if there are any NaN or Inf values in co. If so, estop
	if (!std::isfinite(co.lLeg.motorCurrentA)   ||
	    !std::isfinite(co.lLeg.motorCurrentB)   ||
	    !std::isfinite(co.lLeg.motorCurrentHip) ||
	    !std::isfinite(co.rLeg.motorCurrentA)   ||
	    !std::isfinite(co.rLeg.motorCurrentB)   ||
	    !std::isfinite(co.rLeg.motorCurrentHip))
	{
		return true;
	}

	// If we're not in HALT, set isHalting to false then return false.
	if (rtOps->getStateMachine()->getRtOpsState() != RtOpsState::HALT) {
		isHalting = false;
		return false;
	}

    // Temporary safety -- if halt mode engages, immediately trigger the estop.
    // The amplifiers may be backwards, so letting halt mode engage isn't safe.
	return true;

	// Detect the transition into halt state, so we can initialize the motor rate limits
	if (!isHalting) {
		// Re-initialize the acceptable halt velocity intervals
		lAMinVel = std::min(robotState.lLeg.halfA.rotorVelocity - MOTOR_VEL_MRGN, -MOTOR_VEL_MRGN);
		lAMaxVel = std::max(robotState.lLeg.halfA.rotorVelocity + MOTOR_VEL_MRGN,  MOTOR_VEL_MRGN);
		lBMinVel = std::min(robotState.lLeg.halfB.rotorVelocity - MOTOR_VEL_MRGN, -MOTOR_VEL_MRGN);
		lBMaxVel = std::max(robotState.lLeg.halfB.rotorVelocity + MOTOR_VEL_MRGN,  MOTOR_VEL_MRGN);
		rAMinVel = std::min(robotState.rLeg.halfA.rotorVelocity - MOTOR_VEL_MRGN, -MOTOR_VEL_MRGN);
		rAMaxVel = std::max(robotState.rLeg.halfA.rotorVelocity + MOTOR_VEL_MRGN,  MOTOR_VEL_MRGN);
		rBMinVel = std::min(robotState.rLeg.halfB.rotorVelocity - MOTOR_VEL_MRGN, -MOTOR_VEL_MRGN);
		rBMaxVel = std::max(robotState.rLeg.halfB.rotorVelocity + MOTOR_VEL_MRGN,  MOTOR_VEL_MRGN);

		// Updating isHalting
		isHalting = true;
	}

	// Run the halt check on each motor, and return the results.
	return motorHaltCheck(robotState.lLeg.halfA.rotorVelocity, lAMinVel, lAMaxVel) ||
	       motorHaltCheck(robotState.lLeg.halfB.rotorVelocity, lBMinVel, lBMaxVel) ||
	       motorHaltCheck(robotState.rLeg.halfA.rotorVelocity, rAMinVel, rAMaxVel) ||
	       motorHaltCheck(robotState.rLeg.halfB.rotorVelocity, rBMinVel, rBMaxVel);
}

bool Safety::shouldHalt() {
	atrias_msgs::robot_state robotState = rtOps->getRobotStateHandler()->getRobotState();
	
	// Check for medullas in halt state.
	if (robotState.boomMedullaState        == medulla_state_halt) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::BOOM_MEDULLA_HALT);
		return true;
	}
	
	if (robotState.lLeg.hipMedullaState    == medulla_state_halt) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::LEFT_HIP_MEDULLA_HALT);
		return true;
	}
	
	if (robotState.lLeg.halfA.medullaState == medulla_state_halt) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::LEFT_LEG_A_MEDULLA_HALT);
		return true;
	}
	
	if (robotState.lLeg.halfB.medullaState == medulla_state_halt) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::LEFT_LEG_B_MEDULLA_HALT);
		return true;
	}
	
	if (robotState.rLeg.hipMedullaState    == medulla_state_halt) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::RIGHT_HIP_MEDULLA_HALT);
		return true;
	}
	
	if (robotState.rLeg.halfA.medullaState == medulla_state_halt) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::RIGHT_LEG_A_MEDULLA_HALT);
		return true;
	}
	
	if (robotState.rLeg.halfB.medullaState == medulla_state_halt) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::RIGHT_LEG_B_MEDULLA_HALT);
		return true;
	}
	
	// Disable these safeties if robot configuration is DISABLE.
	if (robotState.robotConfiguration == (RobotConfiguration_t) RobotConfiguration::DISABLE ||
	    robotState.disableSafeties)
		return false;
	
	// These are the predicted stop locations for each motor.
	// We use rotorVelocity since, at this time, we still have occasional spikes in motorVelocity
	double lLegAPred = predictStop(robotState.lLeg.halfA.motorAngle, robotState.lLeg.halfA.rotorVelocity);
	double lLegBPred = predictStop(robotState.lLeg.halfB.motorAngle, robotState.lLeg.halfB.rotorVelocity);
	double rLegAPred = predictStop(robotState.rLeg.halfA.motorAngle, robotState.rLeg.halfA.rotorVelocity);
	double rLegBPred = predictStop(robotState.rLeg.halfB.motorAngle, robotState.rLeg.halfB.rotorVelocity);

	// Check for a collision given this combination of sensor input
	if (checkCollision(lLegAPred, lLegBPred, rLegAPred, rLegBPred))
		return true;

	// Let's also check based purely off the (more reliable) rotor encoders
	lLegAPred = predictStop(robotState.lLeg.halfA.rotorAngle, robotState.lLeg.halfA.rotorVelocity);
	lLegBPred = predictStop(robotState.lLeg.halfB.rotorAngle, robotState.lLeg.halfB.rotorVelocity);
	rLegAPred = predictStop(robotState.rLeg.halfA.rotorAngle, robotState.rLeg.halfA.rotorVelocity);
	rLegBPred = predictStop(robotState.rLeg.halfB.rotorAngle, robotState.rLeg.halfB.rotorVelocity);

	if (checkCollision(lLegAPred, lLegBPred, rLegAPred, rLegBPred))
		return true;

	// If we've made it this far, then we're fine
	return false;
}

bool Safety::checkCollision(double lLegAPred, double lLegBPred, double rLegAPred, double rLegBPred) {
	atrias_msgs::robot_state robotState = rtOps->getRobotStateHandler()->getRobotState();

	// Check if a single motor has exceeded its limits.
	if (lLegAPred < LEG_A_MOTOR_MIN_LOC + LEG_LOC_SAFETY_DISTANCE) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::LEFT_LEG_A_TOO_SMALL);
		return true;
	}
	
	if (lLegAPred > LEG_A_MOTOR_MAX_LOC - LEG_LOC_SAFETY_DISTANCE) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::LEFT_LEG_A_TOO_LARGE);
		return true;
	}
	
	if (lLegBPred < LEG_B_MOTOR_MIN_LOC + LEG_LOC_SAFETY_DISTANCE) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::LEFT_LEG_B_TOO_SMALL);
		return true;
	}
	
	if (lLegBPred > LEG_B_MOTOR_MAX_LOC - LEG_LOC_SAFETY_DISTANCE) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::LEFT_LEG_B_TOO_LARGE);
		return true;
	}
	
	// Check if we've exceeded our leg length limits.
	double lLegAngleDiff = lLegBPred - lLegAPred;
	if (lLegAngleDiff < LEG_LOC_DIFF_MIN) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::LEFT_LEG_TOO_LONG);
		return true;
	}
	
	if (lLegAngleDiff > LEG_LOC_DIFF_MAX) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::LEFT_LEG_TOO_SHORT);
		return true;
	}
	
	// We currently have no checks on the hip, so we don't care about whether
	// or not we have a hip here.
	if (robotState.robotConfiguration == (RobotConfiguration_t) RobotConfiguration::LEFT_LEG_HIP ||
	    robotState.robotConfiguration == (RobotConfiguration_t) RobotConfiguration::LEFT_LEG_NOHIP) {
		return false;
	}
	
	// These are removed for testing of the Medulla halt state. These will be
	// replaced by more advanced safeties anyway.
	if (rLegAPred < LEG_A_MOTOR_MIN_LOC + LEG_LOC_SAFETY_DISTANCE) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::RIGHT_LEG_A_TOO_SMALL);
		return true;
	}
	
	if (rLegAPred > LEG_A_MOTOR_MAX_LOC - LEG_LOC_SAFETY_DISTANCE) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::RIGHT_LEG_A_TOO_LARGE);
		return true;
	}
	
	if (rLegBPred < LEG_B_MOTOR_MIN_LOC + LEG_LOC_SAFETY_DISTANCE) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::RIGHT_LEG_B_TOO_SMALL);
		return true;
	}
	
	if (rLegBPred > LEG_B_MOTOR_MAX_LOC - LEG_LOC_SAFETY_DISTANCE) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::RIGHT_LEG_B_TOO_LARGE);
		return true;
	}
	
	double rLegAngleDiff = rLegBPred - rLegAPred;
	if (rLegAngleDiff < LEG_LOC_DIFF_MIN) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::RIGHT_LEG_TOO_LONG);
		return true;
	}
	
	if (rLegAngleDiff > LEG_LOC_DIFF_MAX) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::RIGHT_LEG_TOO_SHORT);
		return true;
	}
	
	return false;
}

}

}
