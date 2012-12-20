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

double ATCForceHopping::toeHeight(atrias_msgs::robot_state& rs) {
	double lLegLen = cos((rs.lLeg.halfB.legAngle - rs.lLeg.halfA.legAngle) / 2.0);
	double rLegLen = cos((rs.rLeg.halfB.legAngle - rs.rLeg.halfA.legAngle) / 2.0);

	double lLegHgt = rs.position.zPosition - lLegLen * sin((rs.lLeg.halfA.legAngle + rs.lLeg.halfB.legAngle) / 2.0);
	double rLegHgt = rs.position.zPosition - rLegLen * sin((rs.rLeg.halfA.legAngle + rs.rLeg.halfB.legAngle) / 2.0);

	return std::max(lLegHgt, rLegHgt);
}

atrias_msgs::controller_output ATCForceHopping::stateInit(atrias_msgs::robot_state& rs) {
	atrias_msgs::controller_output co;

	lLegFlightGains();
	rLegFlightGains();

	if ((rtOps::RtOpsState) rs.rtOpsState != rtOps::RtOpsState::ENABLED) {
		RobotPos tgt = stateFlight();
		lLegASmoothInit(rs.lLeg.halfA.motorAngle, tgt.lLeg.A,   1.0);
		lLegBSmoothInit(rs.lLeg.halfB.motorAngle, tgt.lLeg.B,   1.0);
		lLegHSmoothInit(rs.lLeg.hip.legBodyAngle, tgt.lLeg.hip, 1.0);
		rLegASmoothInit(rs.rLeg.halfA.motorAngle, tgt.rLeg.A,   1.0);
		rLegBSmoothInit(rs.rLeg.halfB.motorAngle, tgt.rLeg.B,   1.0);
		rLegHSmoothInit(rs.rLeg.hip.legBodyAngle, tgt.rLeg.hip, 1.0);
	}
	
	MotorState lLegADes = lLegASmoothController();
	MotorState lLegBDes = lLegBSmoothController();
	MotorState lLegHDes = lLegHSmoothController();
	MotorState rLegADes = rLegASmoothController();
	MotorState rLegBDes = rLegBSmoothController();
	MotorState rLegHDes = rLegHSmoothController();

	co.lLeg.motorCurrentA   = lLegAController(lLegADes.ang, rs.lLeg.halfA.motorAngle, lLegADes.vel, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB   = lLegBController(lLegBDes.ang, rs.lLeg.halfB.motorAngle, lLegBDes.vel, rs.lLeg.halfB.motorVelocity);
	co.lLeg.motorCurrentHip = lLegHController(lLegHDes.ang, rs.lLeg.hip.legBodyAngle, lLegHDes.vel, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentA   = rLegAController(rLegADes.ang, rs.rLeg.halfA.motorAngle, rLegADes.vel, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB   = rLegBController(rLegBDes.ang, rs.rLeg.halfB.motorAngle, rLegBDes.vel, rs.rLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentHip = rLegHController(rLegHDes.ang, rs.rLeg.hip.legBodyAngle, rLegHDes.vel, rs.rLeg.hip.legBodyVelocity);

	co.command = medulla_state_run;

	if (lLegASmoothFinished.get() &&
	    lLegBSmoothFinished.get() &&
	    lLegHSmoothFinished.get() &&
	    rLegASmoothFinished.get() &&
	    rLegBSmoothFinished.get() &&
	    rLegHSmoothFinished.get()) {
	
		setStateFlight();
	}

	return co;
}

void ATCForceHopping::setStateInit() {
	mode = State::INIT;
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

atrias_msgs::controller_output ATCForceHopping::stateStance(atrias_msgs::robot_state& rs) {
	atrias_msgs::controller_output co;
	co.command = medulla_state_run;

	lLegStanceGains();
	rLegStanceGains();

	double elapsed   = ((double) (rs.timing.controllerTime - stanceStartTime)) / SECOND_IN_NANOSECONDS;
	double duration  = .341;
	double amplitude = 1774.3;

	double tgtForce  = amplitude * sin(M_PI * ((elapsed > duration) ? duration : elapsed) / duration);
	double lDeflDiff = forceDefl(tgtForce / 2.0, rs.lLeg.halfA.legAngle, rs.lLeg.halfB.legAngle);
	double rDeflDiff = forceDefl(tgtForce / 2.0, rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle);

	double lLegSum = rs.lLeg.halfA.legAngle + rs.lLeg.halfB.legAngle;
	double rLegSum = rs.rLeg.halfA.legAngle + rs.rLeg.halfB.legAngle;
	
	RobotPos desState;
	desState.lLeg.A   = (lLegSum + lDeflDiff + rs.lLeg.halfA.legAngle - rs.lLeg.halfB.legAngle) / 2.0;
	desState.lLeg.B   = (lLegSum - lDeflDiff - rs.lLeg.halfA.legAngle + rs.lLeg.halfB.legAngle) / 2.0;
	desState.lLeg.hip = 3.0 * M_PI / 2.0;
	desState.rLeg.A   = (rLegSum + rDeflDiff + rs.rLeg.halfA.legAngle - rs.rLeg.halfB.legAngle) / 2.0;
	desState.rLeg.B   = (rLegSum - rDeflDiff - rs.rLeg.halfA.legAngle + rs.rLeg.halfB.legAngle) / 2.0;
	desState.rLeg.hip = 3.0 * M_PI / 2.0;

	co.lLeg.motorCurrentA   = lLegAController(desState.lLeg.A,           rs.lLeg.halfA.motorAngle,
	                                          rs.lLeg.halfA.legVelocity, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB   = lLegBController(desState.lLeg.B,           rs.lLeg.halfB.motorAngle,
	                                          rs.lLeg.halfB.legVelocity, rs.lLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentA   = rLegAController(desState.rLeg.A,           rs.rLeg.halfA.motorAngle,
	                                          rs.rLeg.halfA.legVelocity, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB   = rLegBController(desState.rLeg.B,           rs.rLeg.halfB.motorAngle,
	                                          rs.rLeg.halfB.legVelocity, rs.rLeg.halfB.motorVelocity);
	co.lLeg.motorCurrentHip = lLegHController(desState.lLeg.hip, rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentHip = rLegHController(desState.rLeg.hip, rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);

	if (elapsed >= duration && toeHeight(rs) > 0.01) {
		setStateFlight();
	}

	logData.elapsed   = elapsed;
	logData.tgtForce  = tgtForce;
	logData.lDeflDiff = lDeflDiff;
	logData.rDeflDiff = rDeflDiff;

	return co;
}

void ATCForceHopping::setStateStance(atrias_msgs::robot_state& rs) {
	stanceStartTime = rs.timing.controllerTime;
	mode = State::STANCE;
}

atrias_msgs::controller_output ATCForceHopping::stateLocked(atrias_msgs::robot_state& rs) {
	atrias_msgs::controller_output co;

	lLegFlightGains();
	rLegFlightGains();

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

	if ((rtOps::RtOpsState) rs.rtOpsState != rtOps::RtOpsState::ENABLED) {
		setStateInit();
	} else if (!guiIn.lockLeg) {
		setStateFlight();
	}
	
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

void ATCForceHopping::lLegStanceGains() {
	lLegAP.set(guiIn.stanceP);
	lLegAD.set(guiIn.stanceD);
	lLegBP.set(guiIn.stanceP);
	lLegBD.set(guiIn.stanceD);
	lLegHP.set(guiIn.hipP);
	lLegHD.set(guiIn.hipD);
}

void ATCForceHopping::rLegStanceGains() {
	rLegAP.set(guiIn.stanceP);
	rLegAD.set(guiIn.stanceD);
	rLegBP.set(guiIn.stanceP);
	rLegBD.set(guiIn.stanceD);
	rLegHP.set(guiIn.hipP);
	rLegHD.set(guiIn.hipD);
}

atrias_msgs::controller_output ATCForceHopping::runController(atrias_msgs::robot_state rs) {
	atrias_msgs::controller_output co;

	switch (mode) {
		case State::INIT:
			co = stateInit(rs);
			break;
		case State::FLIGHT: {
			lLegFlightGains();
			rLegFlightGains();
			RobotPos desState = stateFlight();
			co.lLeg.motorCurrentA   = lLegAController(desState.lLeg.A,   rs.lLeg.halfA.motorAngle, 0, rs.lLeg.halfA.motorVelocity);
			co.lLeg.motorCurrentB   = lLegBController(desState.lLeg.B,   rs.lLeg.halfB.motorAngle, 0, rs.lLeg.halfB.motorVelocity);
			co.lLeg.motorCurrentHip = lLegHController(desState.lLeg.hip, rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
			co.rLeg.motorCurrentA   = rLegAController(desState.rLeg.A,   rs.rLeg.halfA.motorAngle, 0, rs.rLeg.halfA.motorVelocity);
			co.rLeg.motorCurrentB   = rLegBController(desState.rLeg.B,   rs.rLeg.halfB.motorAngle, 0, rs.rLeg.halfB.motorVelocity);
			co.rLeg.motorCurrentHip = rLegHController(desState.rLeg.hip, rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);
			co.command = medulla_state_run;
			if ((rtOps::RtOpsState) rs.rtOpsState != rtOps::RtOpsState::ENABLED) {
				setStateInit();
			} else if (guiIn.lockLeg) {
				setStateLocked();
			} else if (toeHeight(rs) < 0.01) {
				setStateStance(rs);
			}
			break;
		}
		case State::STANCE:
			co = stateStance(rs);
			break;
		case State::LOCKED:
			co = stateLocked(rs);
			break;
		default:
			co.command = medulla_state_error;
			sendEvent(rtOps::RtOpsEvent::CONTROLLER_CUSTOM, (rtOps::RtOpsEventMetadata_t) EventMetadata::BAD_MAIN_STATE);
	}

	// Stuff the msg and push to ROS for logging
	logData.controllerStatus.state = (State_t) mode;
	logData.toeHeight = toeHeight(rs);
	logPort.write(logData);

	if (pubTimer->readyToSend()) {
		guiDataOut.write(logData.controllerStatus);
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

	TaskContext* lLegASmooth = lLegASmoothLoader.load(this, "asc_path_generator", "ASCSmoothPathGenerator");
	if (!lLegASmooth)
		return false;
	lLegASmoothInit       = lLegASmooth->provides("smoothPath")->getOperation("init");
	lLegASmoothController = lLegASmooth->provides("smoothPath")->getOperation("runController");
	lLegASmoothFinished   = lLegASmooth->properties()->getProperty("isFinished");

	TaskContext* lLegBSmooth = lLegBSmoothLoader.load(this, "asc_path_generator", "ASCSmoothPathGenerator");
	if (!lLegBSmooth)
		return false;
	lLegBSmoothInit       = lLegBSmooth->provides("smoothPath")->getOperation("init");
	lLegBSmoothController = lLegBSmooth->provides("smoothPath")->getOperation("runController");
	lLegBSmoothFinished   = lLegBSmooth->properties()->getProperty("isFinished");

	TaskContext* lLegHSmooth = lLegHSmoothLoader.load(this, "asc_path_generator", "ASCSmoothPathGenerator");
	if (!lLegHSmooth)
		return false;
	lLegHSmoothInit       = lLegHSmooth->provides("smoothPath")->getOperation("init");
	lLegHSmoothController = lLegHSmooth->provides("smoothPath")->getOperation("runController");
	lLegHSmoothFinished   = lLegHSmooth->properties()->getProperty("isFinished");

	TaskContext* rLegASmooth = rLegASmoothLoader.load(this, "asc_path_generator", "ASCSmoothPathGenerator");
	if (!rLegASmooth)
		return false;
	rLegASmoothInit       = rLegASmooth->provides("smoothPath")->getOperation("init");
	rLegASmoothController = rLegASmooth->provides("smoothPath")->getOperation("runController");
	rLegASmoothFinished   = rLegASmooth->properties()->getProperty("isFinished");

	TaskContext* rLegBSmooth = rLegBSmoothLoader.load(this, "asc_path_generator", "ASCSmoothPathGenerator");
	if (!rLegBSmooth)
		return false;
	rLegBSmoothInit       = rLegBSmooth->provides("smoothPath")->getOperation("init");
	rLegBSmoothController = rLegBSmooth->provides("smoothPath")->getOperation("runController");
	rLegBSmoothFinished   = rLegBSmooth->properties()->getProperty("isFinished");

	TaskContext* rLegHSmooth = rLegHSmoothLoader.load(this, "asc_path_generator", "ASCSmoothPathGenerator");
	if (!rLegHSmooth)
		return false;
	rLegHSmoothInit       = rLegHSmooth->provides("smoothPath")->getOperation("init");
	rLegHSmoothController = rLegHSmooth->provides("smoothPath")->getOperation("runController");
	rLegHSmoothFinished   = rLegHSmooth->properties()->getProperty("isFinished");

	TaskContext* forceDeflInst = forceDeflLoader.load(this, "asc_force_defl", "ASCForceDefl");
	if (!forceDeflInst)
		return false;
	forceDefl = forceDeflInst->provides("forceDeflection")->getOperation("getDeflectionDiff");

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
