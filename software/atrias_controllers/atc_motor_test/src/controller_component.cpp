/*! \file controller_component.cpp
 *  \author Ryan Van Why
 *  \brief Orocos Component code for the atc_motor_test controller.
 */

#include <atc_motor_test/controller_component.h>

namespace atrias {
namespace controller {

ATCMotorTest::ATCMotorTest(std::string name) :
              RTT::TaskContext(name) {
	
	this->provides("atc")
	->addOperation("runController", &ATCMotorTest::runController, this, ClientThread)
	.doc("Get robot_state from RTOps and return controller output.");

	log(Info) << "[ATCMT] Constructed!" << endlog();
}

atrias_msgs::controller_output ATCMotorTest::runController(atrias_msgs::robot_state rs) {
	atrias_msgs::controller_output co;
	
	// Do nothing unless told otherwise
	co.lLeg.motorCurrentA   = 0.7;
	co.lLeg.motorCurrentB   = 0.0;
	co.lLeg.motorCurrentHip = 0.0;
	co.rLeg.motorCurrentA   = 0.0;
	co.rLeg.motorCurrentB   = 0.0;
	co.rLeg.motorCurrentHip = 0.0;
	
	// Command a run state
	co.command = medulla_state_run;

	// Output for RTOps
	return co;
}

// Don't put control code below here!
bool ATCMotorTest::configureHook() {
	log(Info) << "[ATCMT] configured!" << endlog();
	return true;
}

bool ATCMotorTest::startHook() {
	log(Info) << "[ATCMT] started!" << endlog();
	return true;
}

void ATCMotorTest::stopHook() {
	log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCMotorTest::cleanupHook() {
	log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCMotorTest)

}
}
