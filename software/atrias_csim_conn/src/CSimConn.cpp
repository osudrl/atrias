#include "atrias_csim_conn/CSimConn.h"

namespace atrias {

namespace cSimConn {

CSimConn::CSimConn(std::string name) :
         RTT::TaskContext(name),
         newStateCallback("newStateCallback")
{
	this->provides("connector")
	    ->addOperation("sendControllerOutput", &CSimConn::sendControllerOutput, this, RTT::ClientThread);
	this->requires("rtOps")
	    ->addOperationCaller(newStateCallback);

	// Initialize the state.
	robotState.lLeg.hip.legBodyAngle = robotState.rLeg.hip.legBodyAngle = 1.5 * M_PI;
	robotState.position.bodyPitch    = 1.5 * M_PI;

	robotState.lLeg.halfA.rotorAngle = robotState.rLeg.halfA.rotorAngle =
	robotState.lLeg.halfA.motorAngle = robotState.rLeg.halfA.motorAngle =
	robotState.lLeg.halfA.legAngle   = robotState.rLeg.halfA.legAngle   =
	.25 * M_PI;

	robotState.lLeg.halfB.rotorAngle = robotState.rLeg.halfB.rotorAngle =
	robotState.lLeg.halfB.motorAngle = robotState.rLeg.halfB.motorAngle =
	robotState.lLeg.halfB.legAngle   = robotState.rLeg.halfB.legAngle   =
	.75 * M_PI;
}

atrias_msgs::robot_state_hip CSimConn::simHip(atrias_msgs::robot_state_hip& hip, Hip whichHip) {
	// This represents a net force... it's just in amps, because that's how we know our holding force.
	double netAmps = ((whichHip == Hip::LEFT) ? cOut.lLeg.motorCurrentHip : cOut.rLeg.motorCurrentHip) +
	                 ((whichHip == Hip::LEFT) ? -HIP_HOLD_TORQUE          : HIP_HOLD_TORQUE);
	
	// And the actual net torque.
	double netTorque = HIP_GEARED_TRQ_CONST * netAmps;

	// Calculate acceleration... no friction.
	double accel = netTorque / HIP_INERTIA;

	double newVel = hip.legBodyVelocity + .001 * accel;

	// This hip's minimum position
	double minPos = 1.5 * M_PI - ((whichHip == Hip::LEFT) ? HIP_RELAXED_POS_DIFF  : HIP_EXTENDED_POS_DIFF);
	double maxPos = 1.5 * M_PI + ((whichHip == Hip::LEFT) ? HIP_EXTENDED_POS_DIFF : HIP_RELAXED_POS_DIFF);

	double oldPos = hip.legBodyAngle;
	if (oldPos < minPos && newVel < 0.0)
		newVel = 0.0;
	else if (oldPos > maxPos && newVel > 0.0)
		newVel = 0.0;
	
	atrias_msgs::robot_state_hip out;
	out.legBodyVelocity = newVel;
	out.legBodyAngle    = oldPos + .001 * newVel;
	return out;
}

atrias_msgs::robot_state_legHalf CSimConn::simLegHalf(atrias_msgs::robot_state_legHalf& legHalf, double current, Half half) {
	// Acceleration.
	double accel = ACCEL_PER_AMP * current;

	double newVel = legHalf.motorVelocity + 0.001 * accel;

	// Deceleration due to friction for one simulation step
	double fricDecel = LEG_FRICTION_AMPS * ACCEL_PER_AMP * .001;
	if (newVel > fricDecel)
		newVel -= fricDecel;
	else if (newVel < -fricDecel)
		newVel += fricDecel;
	else
		newVel = 0.0;

	// Previous position
	double pos = legHalf.motorAngle;

	double minpos = (half == Half::A) ? LEG_A_MOTOR_MIN_LOC : LEG_B_MOTOR_MIN_LOC;
	double maxpos = (half == Half::A) ? LEG_A_MOTOR_MAX_LOC : LEG_B_MOTOR_MAX_LOC;
	
	if (pos < minpos && newVel < 0.0)
		newVel = 0.0;
	else if (pos > maxpos && newVel > 0.0)
		newVel = 0.0;
	
	// Compute new position.
	pos += .001 * newVel;
	
	atrias_msgs::robot_state_legHalf out;
	out.rotorAngle    = out.motorAngle    = out.legAngle    = pos;
	out.rotorVelocity = out.motorVelocity = out.legVelocity = newVel;
	return out;
}

bool CSimConn::configureHook() {
	RTT::TaskContext *peer = this->getPeer("atrias_rt");
	if (!peer) {
		log(RTT::Error) << "[CSimConn] Failed to connect to RTOps!" << RTT::endlog();
		return false;
	}
	newStateCallback = peer->provides("rtOps")->getOperation("newStateCallback");
	log(RTT::Info) << "[CSimConn] Connected to RTOps." << RTT::endlog();
	log(RTT::Info) << "[CSimConn] configured!" << RTT::endlog();
	return true;
}

void CSimConn::updateHook() {
	// Increment the time.
	robotState.header.stamp.nsec += CONTROLLER_LOOP_PERIOD_NS;
	robotState.header.stamp.sec  += robotState.header.stamp.nsec / SECOND_IN_NANOSECONDS;
	robotState.header.stamp.nsec %= SECOND_IN_NANOSECONDS;

	// Run the sim.
	robotState.lLeg.hip   = simHip(robotState.lLeg.hip, Hip::LEFT);
	robotState.rLeg.hip   = simHip(robotState.rLeg.hip, Hip::RIGHT);
	robotState.lLeg.halfA = simLegHalf(robotState.lLeg.halfA, cOut.lLeg.motorCurrentA, Half::A);
	robotState.lLeg.halfB = simLegHalf(robotState.lLeg.halfB, cOut.lLeg.motorCurrentB, Half::B);
	robotState.rLeg.halfA = simLegHalf(robotState.rLeg.halfA, cOut.rLeg.motorCurrentA, Half::A);
	robotState.rLeg.halfB = simLegHalf(robotState.rLeg.halfB, cOut.rLeg.motorCurrentB, Half::B);

	newStateCallback(robotState);
}

void CSimConn::sendControllerOutput(atrias_msgs::controller_output controller_output) {
	cOut = controller_output;
	return;
}

ORO_CREATE_COMPONENT(CSimConn)

}

}

// vim: noexpandtab
