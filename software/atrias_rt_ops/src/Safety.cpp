#include "atrias_rt_ops/Safety.h"

namespace atrias {

namespace rtOps {

Safety::Safety(RTOps* rt_ops) {
	rtOps = rt_ops;
}

bool Safety::shouldHalt() {
	atrias_msgs::robot_state robotState = rtOps->getRobotStateHandler()->getRobotState();
	
	// Each of the following checks should send an event (w/ metadata) to the controller
	// manager if satisfied. See issue 98
	
	// Check for medullas in halt state.
	if (robotState.boomMedullaState        == medulla_state_halt)
		return true;
	
	if (robotState.lLeg.hipMedullaState    == medulla_state_halt)
		return true;
	
	if (robotState.lLeg.halfA.medullaState == medulla_state_halt)
		return true;
	
	if (robotState.lLeg.halfB.medullaState == medulla_state_halt)
		return true;
	
	if (robotState.rLeg.hipMedullaState    == medulla_state_halt)
		return true;
	
	if (robotState.rLeg.halfA.medullaState == medulla_state_halt)
		return true;
	
	if (robotState.rLeg.halfB.medullaState == medulla_state_halt)
		return true;
	
	// Check if a single motor has exceeded its limits.
	if (robotState.lLeg.halfA.motorAngle < LEG_A_MOTOR_MIN_LOC + LEG_LOC_SAFETY_DISTANCE)
		return true;
	
	if (robotState.lLeg.halfA.motorAngle > LEG_A_MOTOR_MAX_LOC - LEG_LOC_SAFETY_DISTANCE)
		return true;
	
	if (robotState.lLeg.halfB.motorAngle < LEG_B_MOTOR_MIN_LOC + LEG_LOC_SAFETY_DISTANCE)
		return true;
	
	if (robotState.lLeg.halfB.motorAngle > LEG_B_MOTOR_MAX_LOC - LEG_LOC_SAFETY_DISTANCE)
		return true;
	
	if (robotState.rLeg.halfA.motorAngle < LEG_A_MOTOR_MIN_LOC + LEG_LOC_SAFETY_DISTANCE)
		return true;
	
	if (robotState.rLeg.halfA.motorAngle > LEG_A_MOTOR_MAX_LOC - LEG_LOC_SAFETY_DISTANCE)
		return true;
	
	if (robotState.rLeg.halfB.motorAngle < LEG_B_MOTOR_MIN_LOC + LEG_LOC_SAFETY_DISTANCE)
		return true;
	
	if (robotState.rLeg.halfB.motorAngle > LEG_B_MOTOR_MAX_LOC - LEG_LOC_SAFETY_DISTANCE)
		return true;
	
	// Check if we've exceeded our leg length limits.
	double lLegAngleDiff = robotState.lLeg.halfB.motorAngle - robotState.lLeg.halfA.motorAngle;
	if (lLegAngleDiff < LEG_LOC_DIFF_MIN + LEG_LOC_SAFETY_DISTANCE)
		return true;
	
	if (lLegAngleDiff > LEG_LOC_DIFF_MAX - LEG_LOC_SAFETY_DISTANCE)
		return true;
	
	return false;
}

}

}
