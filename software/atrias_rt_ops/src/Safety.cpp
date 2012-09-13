#include "atrias_rt_ops/Safety.h"

namespace atrias {

namespace rtOps {

Safety::Safety(RTOps* rt_ops) {
	rtOps = rt_ops;
}

bool Safety::shouldHalt() {
	atrias_msgs::robot_state robotState = rtOps->getRobotStateHandler()->getRobotState();
	
	// Check for medullas in halt state.
	if (robotState.boomMedullaState        == medulla_state_halt) {
		log(RTT::Warning) << "Boom medulla halt" << RTT::endlog();
		return true;
	}
	
	if (robotState.lLeg.hipMedullaState    == medulla_state_halt) {
		log(RTT::Warning) << "L leg hip halt" << RTT::endlog();
		return true;
	}
	
	if (robotState.lLeg.halfA.medullaState == medulla_state_halt) {
		log(RTT::Warning) << "L leg A halt" << RTT::endlog();
		return true;
	}
	
	if (robotState.lLeg.halfB.medullaState == medulla_state_halt) {
		log(RTT::Warning) << "L leg B halt" << RTT::endlog();
		return true;
	}
	
	if (robotState.rLeg.hipMedullaState    == medulla_state_halt) {
		log(RTT::Warning) << "R leg hip halt" << RTT::endlog();
		return true;
	}
	
	if (robotState.rLeg.halfA.medullaState == medulla_state_halt) {
		log(RTT::Warning) << "R leg A halt" << RTT::endlog();
		return true;
	}
	
	if (robotState.rLeg.halfB.medullaState == medulla_state_halt) {
		log(RTT::Warning) << "R leg B halt" << RTT::endlog();
		return true;
	}
	
	// Disable these safeties if robot configuration is DISABLE.
	if (robotState.robotConfiguration == (RobotConfiguration_t) RobotConfiguration::DISABLE ||
	    robotState.disableSafeties)
		return false;
	
	// Check if a single motor has exceeded its limits.
	if (robotState.lLeg.halfA.motorAngle < LEG_A_MOTOR_MIN_LOC + LEG_LOC_SAFETY_DISTANCE) {
		log(RTT::Warning) << "Motor A too small" << RTT::endlog();
		return true;
	}
	
	if (robotState.lLeg.halfA.motorAngle > LEG_A_MOTOR_MAX_LOC - LEG_LOC_SAFETY_DISTANCE) {
		log(RTT::Warning) << "Motor A too large" << RTT::endlog();
		return true;
	}
	
	if (robotState.lLeg.halfB.motorAngle < LEG_B_MOTOR_MIN_LOC + LEG_LOC_SAFETY_DISTANCE) {
		log(RTT::Warning) << "Motor B too small" << RTT::endlog();
		return true;
	}
	
	if (robotState.lLeg.halfB.motorAngle > LEG_B_MOTOR_MAX_LOC - LEG_LOC_SAFETY_DISTANCE) {
		log(RTT::Warning) << "Motor B too large" << RTT::endlog();
		return true;
	}
	
	// Check if we've exceeded our leg length limits.
	double lLegAngleDiff = robotState.lLeg.halfB.motorAngle - robotState.lLeg.halfA.motorAngle;
	if (lLegAngleDiff < LEG_LOC_DIFF_MIN + LEG_LOC_SAFETY_DISTANCE) {
		log(RTT::Warning) << "Too long" << RTT::endlog();
		return true;
	}
	
	if (lLegAngleDiff > LEG_LOC_DIFF_MAX - LEG_LOC_SAFETY_DISTANCE) {
		log(RTT::Warning) << "Too short" << RTT::endlog();
		return true;
	}
	
	// We currently have no checks on the hip, so we don't care about whether
	// or not we have a hip here.
	if (robotState.robotConfiguration == (RobotConfiguration_t) RobotConfiguration::LEFT_LEG_HIP ||
	    robotState.robotConfiguration == (RobotConfiguration_t) RobotConfiguration::LEFT_LEG_NOHIP) {
		return false;
	}
	
	if (robotState.rLeg.halfA.motorAngle < LEG_A_MOTOR_MIN_LOC + LEG_LOC_SAFETY_DISTANCE) {
		return true;
	}
	
	if (robotState.rLeg.halfA.motorAngle > LEG_A_MOTOR_MAX_LOC - LEG_LOC_SAFETY_DISTANCE) {
		return true;
	}
	
	if (robotState.rLeg.halfB.motorAngle < LEG_B_MOTOR_MIN_LOC + LEG_LOC_SAFETY_DISTANCE) {
		return true;
	}
	
	if (robotState.rLeg.halfB.motorAngle > LEG_B_MOTOR_MAX_LOC - LEG_LOC_SAFETY_DISTANCE) {
		return true;
	}
	
	double rLegAngleDiff = robotState.rLeg.halfB.motorAngle - robotState.rLeg.halfA.motorAngle;
	if (rLegAngleDiff < LEG_LOC_DIFF_MIN + LEG_LOC_SAFETY_DISTANCE) {
		return true;
	}
	
	if (rLegAngleDiff > LEG_LOC_DIFF_MAX - LEG_LOC_SAFETY_DISTANCE) {
		return true;
	}
	
	return false;
}

}

}
