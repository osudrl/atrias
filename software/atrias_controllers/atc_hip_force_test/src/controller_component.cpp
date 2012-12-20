/*! \file controller_component.cpp
 *  \author Ryan Van Why
 *  \brief Orocos Component code for the atc_hip_force_test controller.
 */

#include <atc_hip_force_test/controller_component.h>

namespace atrias {
namespace controller {

ATCHipForceTest::ATCHipForceTest(std::string name) :
	RTT::TaskContext(name),
	guiDataOut("gui_data_out"),
	guiDataIn("gui_data_in")
{
	this->provides("atc")
	->addOperation("runController", &ATCHipForceTest::runController, this, ClientThread)
	.doc("Get robot_state from RTOps and return controller output.");

	// For the GUI
	addEventPort(guiDataIn);
	addPort(guiDataOut);
	pubTimer = new GuiPublishTimer(20);

	log(Info) << "[ATCMT] Constructed!" << endlog();
}

atrias_msgs::controller_output ATCHipForceTest::runController(atrias_msgs::robot_state rs) {
	// Do nothing unless told otherwise
	co.lLeg.motorCurrentA   = 0.0;
	co.lLeg.motorCurrentB   = 0.0;
	co.lLeg.motorCurrentHip = 0.0;
	co.rLeg.motorCurrentA   = 0.0;
	co.rLeg.motorCurrentB   = 0.0;
	co.rLeg.motorCurrentHip = 0.0;

	// Stuff the msg

	// end control code //

	// Command a run state
	co.command = medulla_state_run;

	// Let the GUI know the controller run state
	// Send data to the GUI
	if (pubTimer->readyToSend())
		guiDataOut.write(guiOut);

	// Output for RTOps
	return co;
}

// Don't put control code below here!
bool ATCHipForceTest::configureHook() {
	log(Info) << "[ATCMT] configured!" << endlog();
	return true;
}

bool ATCHipForceTest::startHook() {
	log(Info) << "[ATCMT] started!" << endlog();
	return true;
}

void ATCHipForceTest::updateHook() {
	guiDataIn.read(guiIn);
}

void ATCHipForceTest::stopHook() {
	log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCHipForceTest::cleanupHook() {
	log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCHipForceTest)

}
}

// vim: noexpandtab
