/*! \file controller_component.cpp
 *  \author Ryan Van Why
 *  \brief Orocos Component code for the atc_subcontroller_test controller.
 */

#include <atc_subcontroller_test/controller_component.h>

namespace atrias {
namespace controller {

ATCSubcontrollerTest::ATCSubcontrollerTest(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log"),
	guiDataOut("gui_data_out"),
	guiDataIn("gui_data_in")
{
	this->provides("atc")
	->addOperation("runController", &ATCSubcontrollerTest::runController, this, ClientThread)
	.doc("Get robot_state from RTOps and return controller output.");

	// For the GUI
	addEventPort(guiDataIn);
	addPort(guiDataOut);
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

atrias_msgs::controller_output ATCSubcontrollerTest::runController(atrias_msgs::robot_state rs) {
	atrias_msgs::controller_output co;
		
	co.lLeg.motorCurrentA = guiIn.des_motor_torque_A;
	co.lLeg.motorCurrentB = guiIn.des_motor_torque_B;
	co.lLeg.motorCurrentHip = guiIn.des_motor_torque_hip;

	// end control code //

	// Command a run state
	co.command = medulla_state_run;

	// Let the GUI know the controller run state
	guiOut.isEnabled = (rs.cmState == (controllerManager::ControllerManagerState_t)controllerManager::ControllerManagerState::CONTROLLER_RUNNING);
	// Send data to the GUI
	if (pubTimer->readyToSend())
		guiDataOut.write(guiOut);

	// Stuff the msg and push to ROS for logging
	logData.desiredState = 0.0;
	logPort.write(logData);

	// Output for RTOps
	return co;
}

// Don't put control code below here!
bool ATCSubcontrollerTest::configureHook() {
	log(Info) << "[ATCMT] configured!" << endlog();
	return true;
}

bool ATCSubcontrollerTest::startHook() {
	log(Info) << "[ATCMT] started!" << endlog();
	return true;
}

void ATCSubcontrollerTest::updateHook() {
	guiDataIn.read(guiIn);
}

void ATCSubcontrollerTest::stopHook() {
	log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCSubcontrollerTest::cleanupHook() {
	log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCSubcontrollerTest)

}
}
