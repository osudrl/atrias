/*! \file controller_component.cpp
 *  \author Ryan Van Why
 *  \brief Orocos Component code for the asc_force_control subcontroller.
 */

#include <asc_force_control/controller_component.h>

namespace atrias {
namespace controller {

ASCForceControl::ASCForceControl(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log")
{
	this->provides("forceControl")
	->addOperation("getTgtState", &ASCForceControl::getTgtState, this, ClientThread)
	.doc("Get the difference in spring controlections required to induce a certain lengthwise force.");

	// Logging
	// Create a port
	addPort(logPort);
	// Connect with buffer size 100000 so we get all data.
	ConnPolicy policy = RTT::ConnPolicy::buffer(100000);
	// Transport type = ROS
	policy.transport = 3;
	// ROS topic name
	policy.name_id = "/" + name + "_log";
	// Construct the stream between the port and ROS topic
	logPort.createStream(policy);

	log(Info) << "[ASCForceControl] Constructed!" << endlog();
}

// Put control code here.
LegState ASCForceControl::getTgtState(robot_state_leg legState, double tgtForce, double dTgtForce) {
	controller_log_data logData;
	LegState                out;

	// Transform matrix
	double mat11  = cos(legState.halfB.legAngle - 0.5 * M_PI);
	double mat12  = sin(legState.halfB.legAngle - 0.5 * M_PI);
	double mat21  = cos(legState.halfA.legAngle - 0.5 * M_PI);
	double mat22  = sin(legState.halfA.legAngle - 0.5 * M_PI);

	// Transform matrix derivative
	double dMat11 = cos(legState.halfB.legAngle);
	double dMat12 = sin(legState.halfB.legAngle);
	double dMat21 = cos(legState.halfA.legAngle);
	double dMat22 = sin(legState.halfA.legAngle);

	// Force vector
	double f1     = tgtForce * cos((legState.halfA.legAngle + legState.halfB.legAngle) / 2.0);
	double f2     = tgtForce * sin((legState.halfA.legAngle + legState.halfB.legAngle) / 2.0);

	// Force vector derivative
	double dF1    = -.5 * tgtForce * (legState.halfA.legVelocity + legState.halfB.legVelocity) *
	                sin(0.5 * (legState.halfA.legAngle + legState.halfB.legAngle)) +
	                dTgtForce * cos(0.5 * (legState.halfA.legAngle + legState.halfB.legAngle));
	double dF2    = 0.5 * tgtForce * (legState.halfA.legVelocity + legState.halfB.legVelocity) *
	                cos(0.5 * (legState.halfA.legAngle + legState.halfB.legAngle)) +
	                dTgtForce * sin(0.5 * (legState.halfA.legAngle + legState.halfB.legAngle));

	// Spring torques
	double tau2   = -.5 * (mat11 * f1 + mat12 * f2);
	double tau1   = -.5 * (mat21 * f1 + mat22 * f2);

	// Spring torque rates
	double dTau2  = -.5 * (dMat11 * f1 + mat11 * dF1 + dMat12 * f2 + mat12 * dF2);
	double dTau1  = -.5 * (dMat21 * f1 + mat21 * dF1 + dMat22 * f2 + mat22 * dF2);

	// Stuff and push the message to ROS for logging
	logData.tgtForce  = tgtForce;
	logData.dTgtForce = dTgtForce;
	logData.aPosOut   = out.A.ang;
	logData.aVelOut   = out.A.vel;
	logData.bPosOut   = out.B.ang;
	logData.bVelOut   = out.B.vel;
	logPort.write(logData);

	// Output for the parent controller
	return out;
}

bool ASCForceControl::configureHook() {
	TaskContext* sprTrqA = torqueDeflALoader.load(this, "asc_spring_torque", "ASCSpringTorque");
	if (!sprTrqA)
		return false;
	sprTrqAConstant = sprTrqA->provides("springTorque")->getOperation("getConstant");
	sprTrqADefl     = sprTrqA->provides("springTorque")->getOperation("getDeflection");

	TaskContext* sprTrqB = torqueDeflBLoader.load(this, "asc_spring_torque", "ASCSpringTorque");
	if (!sprTrqB)
		return false;
	sprTrqBConstant = sprTrqB->provides("springTorque")->getOperation("getConstant");
	sprTrqBDefl     = sprTrqB->provides("springTorque")->getOperation("getDeflection");

	log(Info) << "[ASCForceControl] configured!" << endlog();
	return true;
}

bool ASCForceControl::startHook() {
	log(Info) << "[ASCForceControl] started!" << endlog();
	return true;
}

void ASCForceControl::updateHook() {
}

void ASCForceControl::stopHook() {
	log(Info) << "[ASCForceControl] stopped!" << endlog();
}

void ASCForceControl::cleanupHook() {
	log(Info) << "[ASCForceControl] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCForceControl)

}
}

// vim: noexpandtab
