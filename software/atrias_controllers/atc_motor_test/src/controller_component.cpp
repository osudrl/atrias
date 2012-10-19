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

	// Do nothing unless told otherwise
	co.lLeg.motorCurrentA   = 0.0;
	co.lLeg.motorCurrentB   = 0.0;
	co.lLeg.motorCurrentHip = 0.0;
	co.rLeg.motorCurrentA   = 0.0;
	co.rLeg.motorCurrentB   = 0.0;
	co.rLeg.motorCurrentHip = 0.0;
	
	// Command a run state
	co.command = medulla_state_run;
	
	nextSwitchTime = 0;

	log(Info) << "[ATCMT] Constructed!" << endlog();
}

atrias_msgs::controller_output ATCMotorTest::runController(atrias_msgs::robot_state rs) {
	RTT::os::TimeService::nsecs cur_time = SECOND_IN_NANOSECONDS * rs.header.stamp.sec +
	                                                               rs.header.stamp.nsec;

	if (cur_time > nextSwitchTime) {
		// We need to generate a new output torque and duration.
		double rand_unit = (((double) rand()) - RAND_MAX/2.0) * 2.0 / RAND_MAX;
		double abs_rand  = fabs(rand_unit);
		double sign_rand = SIGN(rand_unit);
		
		co.lLeg.motorCurrentA  = MIN_TORQUE + abs_rand * (MAX_TORQUE - MIN_TORQUE);
		co.lLeg.motorCurrentA *= sign_rand;
		
		nextSwitchTime = cur_time +
			(((double)rand()) * (MAX_DURATION_SEC - MIN_DURATION_SEC) / RAND_MAX + MIN_DURATION_SEC)
			* SECOND_IN_NANOSECONDS;
	}

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
