/*! \file controller_component.cpp
 *  \author Ryan Van Why
 *  \brief Orocos Component code for the asc_toe_decode subcontroller.
 */

#include <asc_toe_decode/controller_component.h>

namespace atrias {
namespace controller {

ASCToeDecode::ASCToeDecode(std::string name) :
	RTT::TaskContext(name),
	filtered_force(0.0),
	onGround(false),
	filter_gain(0.05), // not set yet
	threshold(500.0),  // Also not set yet
	logPort(name + "_log")
{
	this->provides("toeDecode")
	->addOperation("runController", &ASCToeDecode::runController, this, ClientThread)
	.doc("Updates the internal state and returns whether or not the toe's on the ground.");

	this->addProperty("filterGain", filter_gain).doc("The gain of the delay filter.");
	this->addProperty("threshold", threshold).doc("The detection threshold.");

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

	log(Info) << "[ASCToeDecode] Constructed!" << endlog();
}

// Put control code here.
bool ASCToeDecode::runController(uint16_t force) {
	controller_log_data logData;

	// Stuff the msg and push to ROS for logging
	logData.force = force;
	logData.filtered_val = filtered_force;

	// Note: A higher raw force reading means a lower actual force.
	if (onGround && force > filtered_force + threshold) {
		onGround = false;
	} else if (!onGround && force < filtered_force - threshold) {
		onGround = true;
	}

	logData.onGround = onGround;
	logPort.write(logData);

	// Update the filter
	filtered_force += filter_gain * (force - filtered_force);

	// Output for the parent controller
	return onGround;
}

bool ASCToeDecode::configureHook() {
	log(Info) << "[ASCToeDecode] configured!" << endlog();
	return true;
}

bool ASCToeDecode::startHook() {
	log(Info) << "[ASCToeDecode] started!" << endlog();
	return true;
}

void ASCToeDecode::updateHook() {
}

void ASCToeDecode::stopHook() {
	log(Info) << "[ASCToeDecode] stopped!" << endlog();
}

void ASCToeDecode::cleanupHook() {
	log(Info) << "[ASCToeDecode] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCToeDecode)

}
}
