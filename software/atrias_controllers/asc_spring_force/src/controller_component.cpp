/*! \file controller_component.cpp
 *  \author Ryan Van Why
 *  \brief Orocos Component code for the asc_spring_force subcontroller.
 */

#include <asc_spring_force/controller_component.h>

namespace atrias {
namespace controller {

ASCSpringForce::ASCSpringForce(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log")
{
	this->provides("springForce")
	->addOperation("getForce", &ASCSpringForce::getForce, this, ClientThread)
	.doc("Calculates the leg's extension force from its geometry.");

	// Add properties
	this->addProperty("springTorque0Name", springTorque0Name);
	this->addProperty("springTorque1Name", springTorque1Name);

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

	log(Info) << "[ASCSpringForce] Constructed!" << endlog();
}

// Put control code here.
double ASCSpringForce::getForce(double motorAAngle, double legAAngle, double motorBAngle, double legBAngle) {
	controller_log_data logData;
	// Stuff the msg and push to ROS for logging
	logData.motorAAngle = motorAAngle;
	logData.legAAngle   = legAAngle;
	logData.motorBAngle = motorBAngle;
	logData.legBAngle   = legBAngle;
	
	// Compensates for changing leg geometry affecting torque->force relation
	double csc_theta = 1.0 / sin((legBAngle - legAAngle) / 2.0);
	logData.force    = 2.0 * csc_theta *
	                   (springTorque0GetTorque(motorAAngle - legAAngle) -
	                    springTorque1GetTorque(motorBAngle - legBAngle));

	logPort.write(logData);

	// Output for the parent controller
	return logData.force;
}

bool ASCSpringForce::configureHook() {
	// Connect to the subcontrollers
	springTorque0 = this->getPeer(springTorque0Name);
	if (springTorque0)
		springTorque0GetTorque = springTorque0->provides("springTorque")->getOperation("getTorque");

	springTorque1 = this->getPeer(springTorque1Name);
	if (springTorque1)
		springTorque1GetTorque = springTorque1->provides("springTorque")->getOperation("getTorque");

	log(Info) << "[ASCSpringForce] configured!" << endlog();
	return true;
}

bool ASCSpringForce::startHook() {
	log(Info) << "[ASCSpringForce] started!" << endlog();
	return true;
}

void ASCSpringForce::updateHook() {
}

void ASCSpringForce::stopHook() {
	log(Info) << "[ASCSpringForce] stopped!" << endlog();
}

void ASCSpringForce::cleanupHook() {
	log(Info) << "[ASCSpringForce] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCSpringForce)

}
}
