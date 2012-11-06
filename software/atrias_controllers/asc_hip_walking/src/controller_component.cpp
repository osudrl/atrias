/*! \file controller_component.cpp
 *  \author Ryan Van Why
 *  \brief Orocos Component code for the asc_hip_walking subcontroller.
 */

#include <asc_hip_walking/controller_component.h>

namespace atrias {
namespace controller {

ASCHipWalking::ASCHipWalking(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log")
{
	this->provides("hipAngle")
	->addOperation("runController", &ASCHipWalking::runController, this, ClientThread)
	.doc("Run the controller.");

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

	log(Info) << "[ASCHipWalking] Constructed!" << endlog();
}

// Put control code here.
double ASCHipWalking::runController(double legAngle, double boomAngle) {
	double legAngleFromVert = fabs(legAngle - M_PI / 2.0);
	double out = 0.0;

	// Stuff the msg and push to ROS for logging
	controller_log_data logData;
	logData.legAngleIn  = legAngle;
	logData.boomAngleIn = boomAngle;
	logData.hipAngleOut = out;
	logPort.write(logData);

	// Output for the parent controller
	return out;
}

bool ASCHipWalking::configureHook() {
	log(Info) << "[ASCHipWalking] configured!" << endlog();
	return true;
}

bool ASCHipWalking::startHook() {
	log(Info) << "[ASCHipWalking] started!" << endlog();
	return true;
}

void ASCHipWalking::updateHook() {
}

void ASCHipWalking::stopHook() {
	log(Info) << "[ASCHipWalking] stopped!" << endlog();
}

void ASCHipWalking::cleanupHook() {
	log(Info) << "[ASCHipWalking] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCHipWalking)

}
}
