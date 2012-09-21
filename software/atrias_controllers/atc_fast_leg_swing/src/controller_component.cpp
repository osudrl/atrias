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
	this->addProperty("pathGenerator2Name", pathGenerator2Name);
	this->addProperty("pathGenerator3Name", pathGenerator3Name);
	this->addProperty("pathGenerator4Name", pathGenerator4Name);
	this->addProperty("pathGenerator5Name", pathGenerator5Name);
	this->addProperty("pd0Name", pd0Name);
	this->addProperty("pd1Name", pd1Name);
	this->addProperty("pd2Name", pd2Name);
	this->addProperty("pd3Name", pd3Name);
	this->addProperty("pd4Name", pd4Name);
	this->addProperty("pd5Name", pd5Name);
	
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
	// Whether we should extend and retract, as opposed to swinging the legs.
	bool extend = false;
	
	path1ControllerSetPhase((extend) ? 1.0 : 0.0);
	path2ControllerSetPhase((extend) ? 1.0 : 1.0);
	path3ControllerSetPhase((extend) ? 1.0 : 0.0);
	path5ControllerSetPhase((extend) ? 1.0 : 0.0);
	
	MotorState desiredLAState = path0Controller(1, .2);
	desiredLAState.ang += M_PI / 4.0;
	
	MotorState desiredLBState = path1Controller(1, .2);
	desiredLBState.ang += .75 * M_PI;
	
	MotorState desiredLHState = path2Controller((extend) ? 1.0 : 2.0, .1);
	desiredLHState.ang += 1.5 * M_PI;
	
	MotorState desiredRAState = path3Controller(1, .2);
	desiredRAState.ang += M_PI / 4.0;
	
	MotorState desiredRBState = path4Controller(1, .2);
	desiredRBState.ang += .75 * M_PI;
	
	MotorState desiredRHState = path5Controller((extend) ? 1.0 : 2.0, .1);
	desiredRHState.ang += 1.5 * M_PI;
	
	P0.set(500.0);
	D0.set(30.0);
	P1.set(500.0);
	D1.set(30.0);
	P2.set(100.0);
	D2.set(10.0);
	P3.set(500.0);
	D3.set(30.0);
	P4.set(500.0);
	D4.set(30.0);
	P5.set(100.0);
	D5.set(10.0);
	
	co.lLeg.motorCurrentA   = pd0Controller(desiredLAState.ang, rs.lLeg.halfA.motorAngle, desiredLAState.vel, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB   = pd1Controller(desiredLBState.ang, rs.lLeg.halfB.motorAngle, desiredLBState.vel, rs.lLeg.halfB.motorVelocity);
	co.lLeg.motorCurrentHip = pd2Controller(desiredLHState.ang, rs.lLeg.hip.legBodyAngle, desiredLHState.vel, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentA   = pd3Controller(desiredRAState.ang, rs.rLeg.halfA.motorAngle, desiredRAState.vel, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB   = pd4Controller(desiredRBState.ang, rs.rLeg.halfB.motorAngle, desiredRBState.vel, rs.rLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentHip = pd5Controller(desiredRHState.ang, rs.rLeg.hip.legBodyAngle, desiredRHState.vel, rs.rLeg.hip.legBodyVelocity);
	
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
		
	pathGenerator1 = this->getPeer(pathGenerator1Name);
	if (pathGenerator1) {
		path1Controller         = pathGenerator1->provides("parabolaGen")->getOperation("runController");
		path1ControllerSetPhase = pathGenerator1->provides("parabolaGen")->getOperation("setPhase");
	}
	
	pathGenerator2 = this->getPeer(pathGenerator2Name);
	if (pathGenerator2) {
		path2Controller         = pathGenerator2->provides("parabolaGen")->getOperation("runController");
		path2ControllerSetPhase = pathGenerator2->provides("parabolaGen")->getOperation("setPhase");
	}
	
	pathGenerator3 = this->getPeer(pathGenerator3Name);
	if (pathGenerator3) {
		path3Controller         = pathGenerator3->provides("parabolaGen")->getOperation("runController");
		path3ControllerSetPhase = pathGenerator3->provides("parabolaGen")->getOperation("setPhase");
	}
		
	pathGenerator4 = this->getPeer(pathGenerator4Name);
	if (pathGenerator4) {
		path4Controller         = pathGenerator4->provides("parabolaGen")->getOperation("runController");
		path4ControllerSetPhase = pathGenerator4->provides("parabolaGen")->getOperation("setPhase");
	}
	
	pathGenerator5 = this->getPeer(pathGenerator5Name);
	if (pathGenerator5) {
		path5Controller         = pathGenerator5->provides("parabolaGen")->getOperation("runController");
		path5ControllerSetPhase = pathGenerator5->provides("parabolaGen")->getOperation("setPhase");
	}
	
	pd0 = this->getPeer(pd0Name);
	if (pd0)
		pd0Controller = pd0->provides("pd")->getOperation("runController");
	
	pd1 = this->getPeer(pd1Name);
	if (pd1)
		pd1Controller = pd1->provides("pd")->getOperation("runController");
	
	pd2 = this->getPeer(pd2Name);
	if (pd2)
		pd2Controller = pd2->provides("pd")->getOperation("runController");
	
	pd3 = this->getPeer(pd3Name);
	if (pd3)
		pd3Controller = pd3->provides("pd")->getOperation("runController");
	
	pd4 = this->getPeer(pd4Name);
	if (pd4)
		pd4Controller = pd4->provides("pd")->getOperation("runController");
	
	pd5 = this->getPeer(pd5Name);
	if (pd5)
		pd5Controller = pd5->provides("pd")->getOperation("runController");
	
	P0 = pd0->properties()->getProperty("P");
	D0 = pd0->properties()->getProperty("D");
	P1 = pd1->properties()->getProperty("P");
	D1 = pd1->properties()->getProperty("D");
	P2 = pd2->properties()->getProperty("P");
	D2 = pd2->properties()->getProperty("D");
	P3 = pd3->properties()->getProperty("P");
	D3 = pd3->properties()->getProperty("D");
	P4 = pd4->properties()->getProperty("P");
	D4 = pd4->properties()->getProperty("D");
	P5 = pd5->properties()->getProperty("P");
	D5 = pd5->properties()->getProperty("D");
	
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
