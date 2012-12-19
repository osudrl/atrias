/*! \file controller_component.cpp
 *  \author Ryan
 *  \brief Orocos Component code for the asc_hip_force subcontroller.
 */

#include <asc_hip_force/controller_component.h>

namespace atrias {
namespace controller {

ASCHipForce::ASCHipForce(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log")
{
	this->provides("hipForce")
	->addOperation("runController", &ASCHipForce::runController, this, ClientThread)
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

	log(Info) << "[ASCHipForce] Constructed!" << endlog();
}

// Put control code here.
double ASCHipForce::runController(double exampleInput) {
	out = exampleInput;

	// Stuff the msg and push to ROS for logging
	logData.input = exampleInput;
	logData.output = out;
	logPort.write(logData);

	// Output for the parent controller
	return out;
}

bool ASCHipForce::configureHook() {
	log(Info) << "[ASCHipForce] configured!" << endlog();
	return true;
}

bool ASCHipForce::startHook() {
	log(Info) << "[ASCHipForce] started!" << endlog();
	return true;
}

void ASCHipForce::updateHook() {
}

void ASCHipForce::stopHook() {
	log(Info) << "[ASCHipForce] stopped!" << endlog();
}

void ASCHipForce::cleanupHook() {
	log(Info) << "[ASCHipForce] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCHipForce)

}
}

// vim: noexpandtab
