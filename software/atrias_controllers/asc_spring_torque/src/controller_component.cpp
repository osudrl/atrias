/*! \file controller_component.cpp
 *  \author Ryan Van Why
 *  \brief Orocos Component code for the asc_spring_torque subcontroller.
 */

#include <asc_spring_torque/controller_component.h>

namespace atrias {
namespace controller {

ASCSpringTorque::ASCSpringTorque(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log")
{
	this->provides("springTorque")
	->addOperation("getTorque", &ASCSpringTorque::getTorque, this, ClientThread)
	.doc("Calculates the torque from spring deflection");

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

	log(Info) << "[ASCSpringTorque] Constructed!" << endlog();
}

// Put control code here.
double ASCSpringTorque::getTorque(double deflection) {
	controller_log_data logData;

	logData.deflection = deflection;

	double norm = fabs(deflection);
	double sign = (deflection >= 0.0) ? 1.0 : -1.0;

	logData.torque     = sign * linearInterp0GetValue(norm);

	logPort.write(logData);

	// Output for the parent controller
	return logData.torque;
}

bool ASCSpringTorque::configureHook() {
	// Connect to the subcontrollers
	linearInterp0 = this->getPeer(linearInterp0Name);
	if (linearInterp0) {
		linearInterp0InputPoints = linearInterp0->provides("interp")->getOperation("inputPoints");
		linearInterp0GetValue    = linearInterp0->provides("interp")->getOperation("getValue");

		// Input the actual data.
		double torques[NUM_SAMPLES] = SAMPLES;
		linearInterp0InputPoints(torques, NUM_SAMPLES, MIN_SAMPLE_DEFL, MAX_SAMPLE_DEFL);
	}

	log(Info) << "[ASCSpringTorque] configured!" << endlog();
	return true;
}

bool ASCSpringTorque::startHook() {
	log(Info) << "[ASCSpringTorque] started!" << endlog();
	return true;
}

void ASCSpringTorque::updateHook() {
}

void ASCSpringTorque::stopHook() {
	log(Info) << "[ASCSpringTorque] stopped!" << endlog();
}

void ASCSpringTorque::cleanupHook() {
	log(Info) << "[ASCSpringTorque] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCSpringTorque)

}
}
