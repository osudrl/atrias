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
	guiDataIn("gui_data_in"),
	guiDataOut("gui_data_out")
{
	this->provides("atc")
	->addOperation("runController", &ATCForceHopping::runController, this, ClientThread)
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

	mode = State::INIT;

	log(Info) << "[ATCFH] Constructed!" << endlog();
}

atrias_msgs::controller_output ATCForceHopping::stateInit() {
	atrias_msgs::controller_output co;

	lLegFlightGains();
	rLegFlightGains();

	setStateFlight();

	return co;
}

RobotPos ATCForceHopping::stateFlight() {
	RobotPos out;

	MotorAngle desLegState = legToMotorPos(M_PI/2.0, guiIn.flightLegLen);

	out.lLeg.A   = desLegState.A;
	out.lLeg.B   = desLegState.B;
	out.lLeg.hip = 3.0 * M_PI / 2.0;
	out.rLeg.A   = desLegState.A;
	out.rLeg.B   = desLegState.B;
	out.rLeg.hip = 3.0 * M_PI / 2.0;

	return out;
}

void ATCForceHopping::setStateFlight() {
	mode = State::FLIGHT;
}

atrias_msgs::controller_output ATCForceHopping::stateLocked(atrias_msgs::robot_state& rs) {
	atrias_msgs::controller_output co;

	MotorAngle desLegState = legToMotorPos(M_PI/2.0, guiIn.flightLegLen);
	RobotPos desState;
	desState.lLeg.A   = desLegState.A;
	desState.lLeg.B   = desLegState.B;
	desState.lLeg.hip = 3.0 * M_PI / 2.0;
	desState.rLeg.A   = desLegState.A;
	desState.rLeg.B   = desLegState.B;
	desState.rLeg.hip = 3.0 * M_PI / 2.0;

	co.lLeg.motorCurrentA   = lLegAController(desState.lLeg.A,   rs.lLeg.halfA.motorAngle, 0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB   = lLegBController(desState.lLeg.B,   rs.lLeg.halfB.motorAngle, 0, rs.lLeg.halfB.motorVelocity);
	co.lLeg.motorCurrentHip = lLegHController(desState.lLeg.hip, rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentA   = rLegAController(desState.rLeg.A,   rs.rLeg.halfA.motorAngle, 0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB   = rLegBController(desState.rLeg.B,   rs.rLeg.halfB.motorAngle, 0, rs.rLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentHip = rLegHController(desState.rLeg.hip, rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);
	co.command = medulla_state_run;
	
	return co;
}

void ATCForceHopping::setStateLocked() {
	mode = State::LOCKED;
}

void ATCForceHopping::lLegFlightGains() {
	lLegAP.set(guiIn.flightP);
	lLegAD.set(guiIn.flightD);
	lLegBP.set(guiIn.flightP);
	lLegBD.set(guiIn.flightD);
	lLegHP.set(guiIn.hipP);
	lLegHD.set(guiIn.hipD);
}

void ATCForceHopping::rLegFlightGains() {
	rLegAP.set(guiIn.flightP);
	rLegAD.set(guiIn.flightD);
	rLegBP.set(guiIn.flightP);
	rLegBD.set(guiIn.flightD);
	rLegHP.set(guiIn.hipP);
	rLegHD.set(guiIn.hipD);
}

atrias_msgs::controller_output ATCForceHopping::runController(atrias_msgs::robot_state rs) {
	atrias_msgs::controller_output co;

	switch (mode) {
		case State::INIT:
			co = stateInit();
			break;
		case State::FLIGHT: {
			RobotPos desState = stateFlight();
			co.lLeg.motorCurrentA   = lLegAController(desState.lLeg.A,   rs.lLeg.halfA.motorAngle, 0, rs.lLeg.halfA.motorVelocity);
			co.lLeg.motorCurrentB   = lLegBController(desState.lLeg.B,   rs.lLeg.halfB.motorAngle, 0, rs.lLeg.halfB.motorVelocity);
			co.lLeg.motorCurrentHip = lLegHController(desState.lLeg.hip, rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
			co.rLeg.motorCurrentA   = rLegAController(desState.rLeg.A,   rs.rLeg.halfA.motorAngle, 0, rs.rLeg.halfA.motorVelocity);
			co.rLeg.motorCurrentB   = rLegBController(desState.rLeg.B,   rs.rLeg.halfB.motorAngle, 0, rs.rLeg.halfB.motorVelocity);
			co.rLeg.motorCurrentHip = rLegHController(desState.rLeg.hip, rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);
			co.command = medulla_state_run;
			if (guiIn.lockLeg) {
				setStateLocked();
			}
			break;
		}
		case State::STANCE:
			break;
		case State::LOCKED:
			co = stateLocked(rs);
			break;
		default:
			co.command = medulla_state_error;
			sendEvent(rtOps::RtOpsEvent::CONTROLLER_CUSTOM, (rtOps::RtOpsEventMetadata_t) EventMetadata::BAD_MAIN_STATE);
	}

	// Stuff the msg and push to ROS for logging
	controller_log_data logData;
	logData.desiredState = 0.0;
	logPort.write(logData);

	if (pubTimer->readyToSend()) {
		guiOut.state = (State_t) mode;
		guiDataOut.write(guiOut);
	}

	// Output for RTOps
	return co;
}

// Don't put control code below here!
bool ATCForceHopping::configureHook() {
	sendEvent = this->getPeer("atrias_rt")->provides("rtOps")->getOperation("sendEvent");
	if (!sendEvent.ready()) {
		log(RTT::Warning) << "[ATCFH] Failed to acquire sendEvent Operation from RTOps!" << endlog();
	}

	// Load subcontrollers
	TaskContext* pdLLegA = lLegAControllerLoader.load(this, "asc_pd", "ASCPD");
	if (!pdLLegA)
		return false;
	lLegAController = pdLLegA->provides("pd")->getOperation("runController");
	lLegAP = pdLLegA->properties()->getProperty("P");
	lLegAD = pdLLegA->properties()->getProperty("D");

	TaskContext* pdLLegB = lLegBControllerLoader.load(this, "asc_pd", "ASCPD");
	if (!pdLLegB)
		return false;
	lLegBController = pdLLegB->provides("pd")->getOperation("runController");
	lLegBP = pdLLegB->properties()->getProperty("P");
	lLegBD = pdLLegB->properties()->getProperty("D");
	
	TaskContext* pdLLegH = lLegHControllerLoader.load(this, "asc_pd", "ASCPD");
	if (!pdLLegH)
		return false;
	lLegHController = pdLLegH->provides("pd")->getOperation("runController");
	lLegHP = pdLLegH->properties()->getProperty("P");
	lLegHD = pdLLegH->properties()->getProperty("D");

	TaskContext* pdRLegA = rLegAControllerLoader.load(this, "asc_pd", "ASCPD");
	if (!pdRLegA)
		return false;
	rLegAController = pdRLegA->provides("pd")->getOperation("runController");
	rLegAP = pdRLegA->properties()->getProperty("P");
	rLegAD = pdRLegA->properties()->getProperty("D");

	TaskContext* pdRLegB = rLegBControllerLoader.load(this, "asc_pd", "ASCPD");
	if (!pdRLegB)
		return false;
	rLegBController = pdRLegB->provides("pd")->getOperation("runController");
	rLegBP = pdRLegB->properties()->getProperty("P");
	rLegBD = pdRLegB->properties()->getProperty("D");
	
	TaskContext* pdRLegH = rLegHControllerLoader.load(this, "asc_pd", "ASCPD");
	if (!pdRLegH)
		return false;
	rLegHController = pdRLegH->provides("pd")->getOperation("runController");
	rLegHP = pdRLegH->properties()->getProperty("P");
	rLegHD = pdRLegH->properties()->getProperty("D");

	legToMotorPos = this->provides("legToMotorTransforms")->getOperation("posTransform");
	if (!legToMotorPos.ready()) {
		log(RTT::Error) << "[ATCFH] Could not access legToMotorPos's operation!" << endlog();
	}

	log(Info) << "[ATCFH] configured!" << endlog();
	return true;
}

bool ATCForceHopping::startHook() {
	log(Info) << "[ATCFH] started!" << endlog();
	return true;
}

void ATCForceHopping::updateHook() {
	guiDataIn.read(guiIn);
}

void ATCForceHopping::stopHook() {
	log(Info) << "[ATCFH] stopped!" << endlog();
}

void ATCForceHopping::cleanupHook() {
	delete(pubTimer);
	log(Info) << "[ATCFH] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCForceHopping)

}
}

// vim: noexpandtab
