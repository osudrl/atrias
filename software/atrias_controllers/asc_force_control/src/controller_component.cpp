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
double ASCForceDefl::getDeflectionDiff(double force, double legAngleA, double legAngleB) {
	controller_log_data logData;

	// Stuff the msg
	logData.input = force;

	// Calculate the actual torque differential needed.
	// This does not entirely compensate for non-linearities in the springs --
	// it returns a perfect output if symmetrical spring deflections are desired
	// or the springs are perfectly linear.
	logData.output = 2.0 * torqueDefl0GetDefl(force * sin((legAngleB - legAngleA) / 2.0) / 4.0);

	// Push the message to ROS for logging
	logPort.write(logData);

	// Output for the parent controller
	return logData.output;
}

bool ASCForceDefl::configureHook() {
	// Connect to the subcontrollers
	TaskContext* torqueDefl0 = torqueDefl0Loader.load(this, "asc_spring_torque", "ASCSpringTorque");
	if (torqueDefl0)
		torqueDefl0GetDefl = torqueDefl0->provides("springTorque")->getOperation("getDeflection");

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

// vim: noexpandtab
