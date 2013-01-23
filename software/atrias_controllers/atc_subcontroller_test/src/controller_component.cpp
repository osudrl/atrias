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
	co.command = medulla_state_run;

	atrias_msgs::robot_state_leg legState;
	legState.halfA.legAngle    = guiIn.in1;
	legState.halfA.legVelocity = guiIn.in2;
	legState.halfB.legAngle    = guiIn.in3;
	legState.halfB.legVelocity = guiIn.in4;

	//subcontrollerProperty.set(guiIn.in1);

	// Let the GUI know the controller output
	LegState ascOut = subcontrollerOperationCaller(legState, guiIn.in5, guiIn.in6);
	guiOut.out1 = ascOut.A.ang;
	guiOut.out2 = ascOut.A.vel;
	guiOut.out3 = ascOut.B.ang;
	guiOut.out4 = ascOut.B.vel;

	pdAP.set(guiIn.in7);
	pdBP.set(guiIn.in7);
	pdAD.set(guiIn.in8);
	pdBD.set(guiIn.in8);

	co.lLeg.motorCurrentA = pdA(ascOut.A.ang, rs.lLeg.halfA.motorAngle, ascOut.A.vel, rs.lLeg.halfA.rotorVelocity);
	co.lLeg.motorCurrentB = pdB(ascOut.B.ang, rs.lLeg.halfB.motorAngle, ascOut.B.vel, rs.lLeg.halfB.rotorVelocity);

	// Send data to the GUI
	if (pubTimer->readyToSend())
		guiDataOut.write(guiOut);

	logData.input  = guiIn;
	logData.output = guiOut;

	logPort.write(logData);

	// Output for RTOps
	return co;
}

// Don't put control code below here!
bool ATCSubcontrollerTest::configureHook() {
	TaskContext* subcontroller = controllerLoader.load(this, "asc_force_control", "ASCForceControl");
	if (subcontroller) {
		subcontrollerOperationCaller = subcontroller->provides("forceControl")->getOperation("getTgtState");
		//subcontrollerProperty = subcontroller->properties()->getProperty("P");
	}

	TaskContext* pdAInst = pdALoader.load(this, "asc_pd", "ASCPD");
	if (!pdAInst)
		return false;
	pdA  = pdAInst->provides("pd")->getOperation("runController");
	pdAP = pdAInst->properties()->getProperty("P");
	pdAD = pdAInst->properties()->getProperty("D");

	TaskContext* pdBInst = pdBLoader.load(this, "asc_pd", "ASCPD");
	if (!pdBInst)
		return false;
	pdB  = pdBInst->provides("pd")->getOperation("runController");
	pdBP = pdBInst->properties()->getProperty("P");
	pdBD = pdBInst->properties()->getProperty("D");

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
