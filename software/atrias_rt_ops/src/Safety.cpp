#include "atrias_rt_ops/Safety.h"

namespace atrias {

namespace rtOps {

Safety::Safety(RTOps* rt_ops) {
	rtOps = rt_ops;
}

double Safety::predictStop(double pos, double vel) {
	// Derived from basic physics -- assumes constant stopping acceleration.
	return pos + vel * abs(vel) / (2.0 * ACCEL_PER_AMP * AVAIL_HALT_AMPS);
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
	if (rLegAngleDiff < LEG_LOC_DIFF_MIN + LEG_LOC_SAFETY_DISTANCE) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::RIGHT_LEG_TOO_LONG);
		return true;
	}
	
	if (rLegAngleDiff > LEG_LOC_DIFF_MAX - LEG_LOC_SAFETY_DISTANCE) {
		rtOps->getOpsLogger()->sendEvent(RtOpsEvent::SAFETY, (RtOpsEventMetadata_t) RtOpsEventSafetyMetadata::RIGHT_LEG_TOO_SHORT);
		return true;
	}
	
	return false;
}

}

}
