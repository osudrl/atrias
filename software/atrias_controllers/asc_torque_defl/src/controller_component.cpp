/*! \file controller_component.cpp
 *  \author Ryan Van Why
 *  \brief Orocos Component code for the asc_torque_defl subcontroller.
 */

#include <asc_torque_defl/controller_component.h>

namespace atrias {
namespace controller {

ASCTorqueDefl::ASCTorqueDefl(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log")
{
	this->provides("torqueDeflection")
	->addOperation("getDefl", &ASCTorqueDefl::getDefl, this, ClientThread)
	.doc("Calculates the required spring deflection to achieve a given torque.");

	// Add properties
	this->addProperty("linearInterp0Name", linearInterp0Name);

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

	log(Info) << "[ASCTorqueDefl] Constructed!" << endlog();
}

// Put control code here.
double ASCTorqueDefl::getDefl(double torque) {
	controller_log_data logData;

	// Stuff the msg and push to ROS for logging
	logData.torque     = torque;

	double norm = fabs(torque);
	double sign = (torque >= 0.0) ? 1.0 : -1.0;
	logData.deflection = sign * linearInterp0GetValue(norm); // Todo: This
	logPort.write(logData);

	// Output for the parent controller
	return logData.deflection;
}

bool ASCTorqueDefl::configureHook() {
	// Connect to the subcontrollers
	linearInterp0 = this->getPeer(linearInterp0Name);
	if (linearInterp0) {
		linearInterp0InputPoints = linearInterp0->provides("interp")->getOperation("inputPoints");
		linearInterp0GetValue    = linearInterp0->provides("interp")->getOperation("getValue");

		double torques[NUM_SAMPLES] = SAMPLES;
		linearInterp0InputPoints(torques, NUM_SAMPLES, MIN_SAMPLE_TRQ, MAX_SAMPLE_TRQ);
	}

	log(Info) << "[ASCTorqueDefl] configured!" << endlog();
	return true;
}

bool ASCTorqueDefl::startHook() {
	log(Info) << "[ASCTorqueDefl] started!" << endlog();
	return true;
}

void ASCTorqueDefl::updateHook() {
}

void ASCTorqueDefl::stopHook() {
	log(Info) << "[ASCTorqueDefl] stopped!" << endlog();
}

void ASCTorqueDefl::cleanupHook() {
	log(Info) << "[ASCTorqueDefl] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCTorqueDefl)

}
}
