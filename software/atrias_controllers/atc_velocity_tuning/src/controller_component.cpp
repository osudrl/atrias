/*! \file controller_component.cpp
 *  \author Ryan Van Why
 *  \brief Orocos Component code for the atc_velocity_tuning controller.
 */

#include <atc_velocity_tuning/controller_component.h>

namespace atrias {
namespace controller {

ATCVelocityTuning::ATCVelocityTuning(std::string name) :
	RTT::TaskContext(name),
	guiDataIn("gui_data_in")
{
	this->provides("atc")
	->addOperation("runController", &ATCVelocityTuning::runController, this, ClientThread)
	.doc("Get robot_state from RTOps and return controller output.");

	// Add properties
	this->addProperty("pd0Name", pd0Name);

	// For the GUI
	addEventPort(guiDataIn);
	
	cur_dir = 1;
	
	log(Info) << "[ATCMT] Constructed!" << endlog();
}

atrias_msgs::controller_output ATCVelocityTuning::runController(atrias_msgs::robot_state rs) {
	atrias_msgs::controller_output co;
	P0.set(guiIn.Kp);

	if (guiIn.relayMode == 1 && abs(rs.rLeg.halfA.rotorAngle - rs.rLeg.halfA.motorAngle) > 0.1) {
		co.command = medulla_state_error;
	} else {
		co.command = medulla_state_run;
	}

	if (guiIn.relayMode == 0 || guiIn.relayMode == 1) {
		// Set motorAngle based on the correct sensor.
		double motorAngle = (guiIn.relayMode == 0) ? rs.rLeg.halfA.motorAngle : rs.rLeg.halfA.rotorAngle;

		if (motorAngle > guiIn.maxPos)
			cur_dir = -1;
		if (motorAngle < guiIn.minPos)
			cur_dir = 1;
	} else if (guiIn.relayMode == 2) {
		// Forward
		cur_dir = 1;
	} else {
		// Reverse
		cur_dir = -1;
	}
	
	co.rLeg.motorCurrentA   = pd0RunController(cur_dir * guiIn.desVel,
	                                           (guiIn.sensor) ? rs.rLeg.halfA.rotorVelocity :
	                                                            rs.rLeg.halfA.motorVelocity,
	                                           0.0, 0.0);
	co.lLeg.motorCurrentA   = 0.0;
	co.lLeg.motorCurrentB   = 0.0;
	co.lLeg.motorCurrentHip = 0.0;
	co.rLeg.motorCurrentB   = 0.0;
	co.rLeg.motorCurrentHip = 0.0;

	// Output for RTOps
	return co;
}

// Don't put control code below here!
bool ATCVelocityTuning::configureHook() {
	// Connect to the subcontrollers
	pd0 = this->getPeer(pd0Name);
	if (pd0)
		pd0RunController = pd0->provides("pd")->getOperation("runController");

	// Get references to subcontroller component properties
	D0 = pd0->properties()->getProperty("D");
	P0 = pd0->properties()->getProperty("P");
	
	D0.set(0.0);

	log(Info) << "[ATCMT] configured!" << endlog();
	return true;
}

bool ATCVelocityTuning::startHook() {
	log(Info) << "[ATCMT] started!" << endlog();
	return true;
}

void ATCVelocityTuning::updateHook() {
	guiDataIn.read(guiIn);
}

void ATCVelocityTuning::stopHook() {
	log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCVelocityTuning::cleanupHook() {
	log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCVelocityTuning)

}
}
