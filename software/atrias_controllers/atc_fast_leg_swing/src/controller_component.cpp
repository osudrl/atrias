/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_fast_leg_swing controller.
 */

#include <atc_fast_leg_swing/controller_component.h>

namespace atrias {
namespace controller {

ATCFastLegSwing::ATCFastLegSwing(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log"),
	guiDataIn("gui_data_in")
{
	this->provides("atc")
	->addOperation("runController", &ATCFastLegSwing::runController, this, ClientThread)
	.doc("Get robot_state from RTOps and return controller output.");

	// Add properties
	this->addProperty("pathGenerator0Name", pathGenerator0Name);
	this->addProperty("pathGenerator1Name", pathGenerator1Name);
	this->addProperty("pd0Name", pd0Name)
		.doc("Name of 0th PD subcontroller.");
	this->addProperty("pd1Name", pd1Name)
		.doc("Name of 1th PD subcontroller.");
	
	// For the GUI
	addEventPort(guiDataIn);
	pubTimer = new GuiPublishTimer(20);

	// Logging
	// Create a port
	addPort(logPort);
	// Unbuffered
	ConnPolicy policy = RTT::ConnPolicy::data();
	// Transport type = ROS
	policy.transport = 3;
	// ROS topic name
	policy.name_id = "/" + name + "_log";
	// Construct the stream between the port and ROS topic
	logPort.createStream(policy);

	co.command = medulla_state_run;

	log(Info) << "[ATCFLS] Constructed!" << endlog();
}

atrias_msgs::controller_output ATCFastLegSwing::runController(atrias_msgs::robot_state rs) {
	MotorState desiredLAState = path0Controller(3, .1);
	desiredLAState.ang += M_PI / 4.0;
	
	MotorState desiredLBState = {0.0, 0.0};
	desiredLBState.ang += .75 * M_PI;
	
	P0.set(1000.0);
	D0.set(50.0);
	P1.set(1000.0);
	D1.set(50.0);
	
	co.lLeg.motorCurrentA   = pd0Controller(desiredLAState.ang, rs.lLeg.halfA.motorAngle, desiredLAState.vel, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB   = pd1Controller(desiredLBState.ang, rs.lLeg.halfB.motorAngle, desiredLBState.vel, rs.lLeg.halfB.motorVelocity);
	co.lLeg.motorCurrentHip = 0.0;
	co.rLeg.motorCurrentA   = 0.0;
	co.rLeg.motorCurrentB   = 0.0;
	co.rLeg.motorCurrentHip = 0.0;
	
	// Stuff the msg and push to ROS for logging
	logData.desiredState = 0.0;
	logPort.write(logData);

	// Output for RTOps
	return co;
}

// Don't put control code below here!
bool ATCFastLegSwing::configureHook() {
	// Connect to the subcontrollers
	// Get references to subcontroller component properties
	pathGenerator0 = this->getPeer(pathGenerator0Name);
	if (pathGenerator0)
		path0Controller = pathGenerator0->provides("parabolaGen")->getOperation("runController");
	
	pd0 = this->getPeer(pd0Name);
	if (pd0)
		pd0Controller = pd0->provides("pd")->getOperation("runController");
	
	pd1 = this->getPeer(pd1Name);
	if (pd1)
		pd1Controller = pd1->provides("pd")->getOperation("runController");
	
	P0 = pd0->properties()->getProperty("P");
	D0 = pd0->properties()->getProperty("D");
	P1 = pd1->properties()->getProperty("P");
	D1 = pd1->properties()->getProperty("D");
	
	log(Info) << "[ATCFLS] configured!" << endlog();
	return true;
}

bool ATCFastLegSwing::startHook() {
	log(Info) << "[ATCFLS] started!" << endlog();
	return true;
}

void ATCFastLegSwing::updateHook() {
	guiDataIn.read(guiIn);
}

void ATCFastLegSwing::stopHook() {
	log(Info) << "[ATCFLS] stopped!" << endlog();
}

void ATCFastLegSwing::cleanupHook() {
	log(Info) << "[ATCFLS] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCFastLegSwing)

}
}
