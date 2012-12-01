/*! \file controller_component.cpp
 *  \author Ryan Van Why
 *  \brief Orocos Component code for the atc_force_hopping controller.
 */

#include <atc_force_hopping/controller_component.h>

namespace atrias {
namespace controller {

ATCForceHopping::ATCForceHopping(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log"),
	guiDataIn("gui_data_in")
{
	this->provides("atc")
	->addOperation("runController", &ATCForceHopping::runController, this, ClientThread)
	.doc("Get robot_state from RTOps and return controller output.");

	// For the GUI
	addEventPort(guiDataIn);
	pubTimer = new GuiPublishTimer(20);

	// Logging
	// Create a port
	addPort(logPort);
	// Buffer port so we capture all data.
	ConnPolicy policy = RTT::ConnPolicy::buffer(100000);
	// Transport type = ROS
	policy.transport = 3;
	// ROS topic name
	policy.name_id = "/" + name + "_log";
	// Construct the stream between the port and ROS topic
	logPort.createStream(policy);

	log(Info) << "[ATCMT] Constructed!" << endlog();
}

atrias_msgs::controller_output ATCForceHopping::runController(atrias_msgs::robot_state rs) {
	// Do nothing unless told otherwise
	co.lLeg.motorCurrentA   = 0.0;
	co.lLeg.motorCurrentB   = 0.0;
	co.lLeg.motorCurrentHip = 0.0;
	co.rLeg.motorCurrentA   = 0.0;
	co.rLeg.motorCurrentB   = 0.0;
	co.rLeg.motorCurrentHip = 0.0;

	// Only run the controller when we're enabled
	if ((uint8_t)rs.cmState != (uint8_t) controllerManager::RtOpsCommand::ENABLE)
		return co;

	// begin control code //

	// Stuff the msg
	co.lLeg.motorCurrentA = guiIn.des_motor_torque_A;
	co.lLeg.motorCurrentB = guiIn.des_motor_torque_B;
	co.lLeg.motorCurrentHip = guiIn.des_motor_torque_hip;

	// end control code //

	// Command a run state
	co.command = medulla_state_run;

	// Stuff the msg and push to ROS for logging
	logData.desiredState = 0.0;
	logPort.write(logData);

	// Output for RTOps
	return co;
}

// Don't put control code below here!
bool ATCForceHopping::configureHook() {
	log(Info) << "[ATCMT] configured!" << endlog();
	return true;
}

bool ATCForceHopping::startHook() {
	log(Info) << "[ATCMT] started!" << endlog();
	return true;
}

void ATCForceHopping::updateHook() {
	guiDataIn.read(guiIn);
}

void ATCForceHopping::stopHook() {
	log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCForceHopping::cleanupHook() {
	log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCForceHopping)

}
}
