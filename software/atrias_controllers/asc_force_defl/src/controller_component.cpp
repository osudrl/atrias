/*! \file controller_component.cpp
 *  \author Ryan Van Why
 *  \brief Orocos Component code for the asc_force_defl subcontroller.
 */

#include <asc_force_defl/controller_component.h>

namespace atrias {
namespace controller {

ASCForceDefl::ASCForceDefl(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log")
{
	this->provides("forceDeflection")
	->addOperation("getDeflectionDiff", &ASCForceDefl::getDeflectionDiff, this, ClientThread)
	.doc("Get the difference in spring deflections required to induce a certain lengthwise force.");

	// Add properties
	this->addProperty("torqueDefl0Name", torqueDefl0Name);

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

	log(Info) << "[ASCForceDefl] Constructed!" << endlog();
}

// Put control code here.
double ASCForceDefl::getDeflectionDiff(double force) {
	controller_log_data logData;

	// Stuff the msg and push to ROS for logging
	logData.input = exampleInput;
	logData.output = out;
	logPort.write(logData);

	// Output for the parent controller
	return out;
}

bool ASCForceDefl::configureHook() {
	// Connect to the subcontrollers
	torqueDefl0 = this->getPeer(torqueDefl0Name);
	if (torqueDefl0)
		torqueDefl0GetDefl = torqueDefl0->provides("torqueDeflection")->getOperation("getDefl");

	// Get references to subcontroller component properties
	linearInterp0Name0 = torqueDefl0->properties()->getProperty("linearInterp0Name");

	log(Info) << "[ASCForceDefl] configured!" << endlog();
	return true;
}

bool ASCForceDefl::startHook() {
	log(Info) << "[ASCForceDefl] started!" << endlog();
	return true;
}

void ASCForceDefl::updateHook() {
}

void ASCForceDefl::stopHook() {
	log(Info) << "[ASCForceDefl] stopped!" << endlog();
}

void ASCForceDefl::cleanupHook() {
	log(Info) << "[ASCForceDefl] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCForceDefl)

}
}
