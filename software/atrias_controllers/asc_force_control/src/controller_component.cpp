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
