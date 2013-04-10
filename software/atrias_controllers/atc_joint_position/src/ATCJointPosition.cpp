#include "atc_joint_position/ATCJointPosition.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// This constructor call is much simpler.
ATCJointPosition::ATCJointPosition(string name) :
	ATC(name),
	pdLA(this, "pdLA"),
	pdLB(this, "pdLB"),
	pdLH(this, "pdLH"),
	pdRA(this, "pdRA"),
	pdRB(this, "pdRB"),
	pdRH(this, "pdRH"),
	rateLimLA(this, "rateLimLA"),
	rateLimLB(this, "rateLimLB"),
	rateLimLH(this, "rateLimLH"),
	rateLimRA(this, "rateLimRA"),
	rateLimRB(this, "rateLimRB"),
	rateLimRH(this, "rateLimRH"),
	commonToolkit(this, "commonToolkit"),
	hipKine(this, "hipKine")
{
	// Nothing to see here
}

void ATCJointPosition::controller() {
	// Reset rate limiters while disabled to get a smooth init.
	if (!isEnabled()) {
		logOut.lHipFast = false;
		logOut.rHipFast = false;
		rateLimLA.reset(rs.lLeg.halfA.rotorAngle);
		rateLimLB.reset(rs.lLeg.halfB.rotorAngle);
		rateLimLH.reset(rs.lLeg.hip.legBodyAngle);
		rateLimRA.reset(rs.rLeg.halfA.rotorAngle);
		rateLimRB.reset(rs.rLeg.halfB.rotorAngle);
		rateLimRH.reset(rs.rLeg.hip.legBodyAngle);
	}

	// Hip angle code
	double lTgtAng;
	double rTgtAng;
	if (guiIn.vertical) {
		// Set desired hip angles for inverse kinematics mode
		LeftRight toePositions;
		toePositions.left  = 2.15;
		toePositions.right = 2.45;
		double lDesAng;
		double rDesAng;
		std::tie(lDesAng, rDesAng) = hipKine.iKine(toePositions, rs.lLeg, rs.rLeg, rs.position);

		// Move hips to these locations
		lTgtAng = rateLimLH(lDesAng, guiIn.maxHipSpd);
		rTgtAng = rateLimRH(rDesAng, guiIn.maxHipSpd);
		// Increase rate limit if following kinematic constraint
		if (logOut.lHipFast) {
			lTgtAng = rateLimLH.reset(lDesAng);
		} else {
			lTgtAng = rateLimLH(lDesAng, guiIn.maxHipSpd);
			if (lTgtAng == lDesAng)
				logOut.lHipFast = true;
		}
		if (logOut.rHipFast) {
			rTgtAng = rateLimRH.reset(rDesAng);
		} else {
			rTgtAng = rateLimRH(rDesAng, guiIn.maxHipSpd);
			if (rTgtAng == rDesAng)
				logOut.rHipFast = true;
		}
	} else {
		// Enable slow motion if set in the GUI
		if (guiIn.smoothHipMotion) {
			logOut.lHipFast = logOut.rHipFast = false;
			lTgtAng = rateLimLH(guiIn.a_hl, guiIn.maxHipSpd);
			rTgtAng = rateLimRH(guiIn.a_hr, guiIn.maxHipSpd);
		} else {
			logOut.lHipFast = logOut.rHipFast = true;
			lTgtAng = rateLimLH.reset(guiIn.a_hl);
			rTgtAng = rateLimRH.reset(guiIn.a_hr);
		}
	}
	// And command the hip torques
	pdLH.P = guiIn.p_hl;
	pdLH.D = guiIn.d_hl;
	pdRH.P = guiIn.p_hr;
	pdRH.D = guiIn.d_hr;
	co.lLeg.motorCurrentHip = pdLH(lTgtAng, rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentHip = pdRH(rTgtAng, rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);

	// Leg code
	AB lOut = runLeg(rs.lLeg, guiIn.p_ll, guiIn.d_ll, guiIn.a_ll, guiIn.l_ll, rateLimLA, rateLimLB, pdLA, pdLB);
	AB rOut = runLeg(rs.rLeg, guiIn.p_lr, guiIn.d_lr, guiIn.a_lr, guiIn.l_lr, rateLimRA, rateLimRB, pdRA, pdRB);

	co.lLeg.motorCurrentA = lOut.A;
	co.lLeg.motorCurrentB = lOut.B;
	co.rLeg.motorCurrentA = rOut.A;
	co.rLeg.motorCurrentB = rOut.B;
}

AB ATCJointPosition::runLeg(robot_state_leg &leg, double P, double D, double ang, double len,
	ASCRateLimit &rateLimA, ASCRateLimit &rateLimB, ASCPD &pdA, ASCPD &pdB)
{
	// Get desired motor positions
	double tgtA;
	double tgtB;
	std::tie(tgtA, tgtB) = commonToolkit.legPos2MotorPos(ang, len);

	// Smooth out motion, if enabled
	if (guiIn.smoothLegMotion) {
		tgtA = rateLimA(tgtA, guiIn.maxLegSpd);
		tgtB = rateLimB(tgtB, guiIn.maxLegSpd);
	} else {
		rateLimA.reset(tgtA);
		rateLimB.reset(tgtB);
	}

	// Send the commands
	pdA.P = pdB.P = P;
	pdA.D = pdB.D = D;
	AB out;
	out.A = pdA(tgtA, leg.halfA.rotorAngle, 0, leg.halfA.rotorVelocity);
	out.B = pdB(tgtB, leg.halfB.rotorAngle, 0, leg.halfB.rotorVelocity);

	return out;
}

// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCJointPosition)

}
}

// vim: noexpandtab
