/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the parabola path generator subcontroller.
 */

#include <asc_path_generator/parabola_path_generator.h>

namespace atrias {
namespace controller {

ASCParabolaPathGenerator::ASCParabolaPathGenerator(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log")
{
	this->provides("parabolaGen")
	->addOperation("runController", &ASCParabolaPathGenerator::runController, this, ClientThread)
	.doc("Run the controller.");
	this->provides("parabolaGen")
	->addOperation("reset", &ASCParabolaPathGenerator::reset, this, ClientThread);
	this->provides("parabolaGen")
	->addOperation("setPhase", &ASCParabolaPathGenerator::setPhase, this, ClientThread);

	// For logging
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

	// Setup the controller
	accumulator = 0.0;
	phase = 0.0;

	log(Info) << "[ASCPG] Constructed!" << endlog();
}

void ASCParabolaPathGenerator::reset(void)
{
	// Reset the accumulator
	accumulator = 0.0;
}

void ASCParabolaPathGenerator::setPhase(double _phase)
{
	// Set the phase
	phase = _phase;
}

MotorState ASCParabolaPathGenerator::runController(double frequency, double amplitude)
{
	// The parabola input
	accumulator += 0.001;
	// Wrap the parabola input when the input value gets to be a multiple of 2.0*pi
	if (accumulator >= (1.0/frequency))
		accumulator -= (1.0/frequency);

	// Stuff the msg and push to ROS for logging
	logData.ang = parabolaOut.ang;
	logData.vel = parabolaOut.vel;
	logPort.write(logData);

	return parabolaOut;
}

bool ASCParabolaPathGenerator::configureHook() {
	log(Info) << "[ASCPG] configured!" << endlog();
	return true;
}

bool ASCParabolaPathGenerator::startHook() {
	log(Info) << "[ASCPG] started!" << endlog();
	return true;
}

void ASCParabolaPathGenerator::updateHook() {
}

void ASCParabolaPathGenerator::stopHook() {
	log(Info) << "[ASCPG] stopped!" << endlog();
}

void ASCParabolaPathGenerator::cleanupHook() {
	log(Info) << "[ASCPG] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCParabolaPathGenerator)

}
}
