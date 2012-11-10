/*! \file controller_component.cpp
 *  \author Ryan Van Why
 *  \brief Orocos Component code for the asc_linear_interp subcontroller.
 */

#include <asc_linear_interp/controller_component.h>

namespace atrias {
namespace controller {

ASCLinearInterp::ASCLinearInterp(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log")
{
	this->provides("interp")
		->addOperation("getValue", &ASCLinearInterp::runController, this, ClientThread)
		.doc("Return the interpolated value at some x value");
	this->provides("interp")
		->addOperation("inputPoints", &ASCLinearInterp::inputPoints, this, ClientThread)
		.doc("Input the points and interval in which to interpolate.");

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

	log(Info) << "[ASCLinearInterp] Constructed!" << endlog();
}

// Put control code here.
double ASCLinearInterp::runController(double input) {
	double out = 0.0;

	if (input <= a) {
		out = values[0];
	} else if (input >= b) {
		out = values[numValues];
	} else {
		double mapped = (input - a) * (numValues - 1) / (b - a);
		double firstSample = values[(int) floor(mapped)];
		// Note: input < b by now, so mapped < numValues
		double secondSample = values[(int) floor(mapped) + 1];
		double norm = fmod(mapped, 1);
		out = norm * secondSample + (1.0 - norm) * firstSample;
	}

	// Stuff the msg and push to ROS for logging
	controller_log_data logData;
	logData.input  = input;
	logData.output = out;
	logPort.write(logData);

	// Output for the parent controller
	return out;
}

void ASCLinearInterp::inputPoints(double samples[], int numSamples, double start, double end) {
	values    = samples;
	numValues = numSamples;
	a         = (start < end) ? start : end;
	b         = (start > end) ? start : end;
}

bool ASCLinearInterp::configureHook() {
	log(Info) << "[ASCLinearInterp] configured!" << endlog();
	return true;
}

bool ASCLinearInterp::startHook() {
	log(Info) << "[ASCLinearInterp] started!" << endlog();
	return true;
}

void ASCLinearInterp::updateHook() {
}

void ASCLinearInterp::stopHook() {
	log(Info) << "[ASCLinearInterp] stopped!" << endlog();
}

void ASCLinearInterp::cleanupHook() {
	log(Info) << "[ASCLinearInterp] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCLinearInterp)

}
}
