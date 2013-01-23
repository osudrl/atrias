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
	atrias_msgs::controller_output co;

	flightP.set(guiIn.flightP);
	flightD.set(guiIn.flightD);
	stanceP.set(guiIn.stanceP);
	stanceD.set(guiIn.stanceD);
	toeFilterGain.set(guiIn.toeFilterGain);
	toeThreshold.set(guiIn.toeThreshold);
	HP.set(guiIn.flightP);
	HD.set(guiIn.flightD);
	AP.set(guiIn.legP);
	AD.set(guiIn.legD);
	BP.set(guiIn.legP);
	BD.set(guiIn.legD);

	MotorAngle flightLegPos = legToMotorPos(M_PI / 2.0, guiIn.legLen);
	
	if ((rtOps::RtOpsState) rs.rtOpsState != rtOps::RtOpsState::ENABLED) {
		// Only set this while disabled.
		activeLeg = guiIn.activeLeg;

		// Feed the smooth path controllers.
		smoothHInit((activeLeg == 0) ? rs.lLeg.hip.legBodyAngle : rs.rLeg.hip.legBodyAngle, 1.5 * M_PI,     1.0);
		smoothAInit((activeLeg == 0) ? rs.lLeg.halfA.motorAngle : rs.rLeg.halfA.motorAngle, flightLegPos.A, 1.0);
		smoothBInit((activeLeg == 0) ? rs.lLeg.halfB.motorAngle : rs.rLeg.halfB.motorAngle, flightLegPos.B, 1.0);
	}

	// Assign the active and inactive legs and zero out inactive torques.
	atrias_msgs::controller_output_leg *coActiveLeg;
	atrias_msgs::controller_output_leg *coInactiveLeg;
	atrias_msgs::robot_state_leg       *rsActiveLeg;
	switch (activeLeg) {
		case 0: // Left
			coActiveLeg   = &(co.lLeg);
			coInactiveLeg = &(co.rLeg);
			rsActiveLeg   = &(rs.lLeg);
			break;
		case 1: // right
			coActiveLeg   = &(co.rLeg);
			coInactiveLeg = &(co.lLeg);
			rsActiveLeg   = &(rs.rLeg);
			break;
		default:
			co.command = medulla_state_error;
			return co;
	}
	coInactiveLeg->motorCurrentA   = 0.0;
	coInactiveLeg->motorCurrentB   = 0.0;
	coInactiveLeg->motorCurrentHip = 0.0;
	
	if (!smoothHDone || !smoothADone || !smoothBDone) {
		// Smoothly initialize
		MotorState desH = smoothHCont();
		MotorState desA = smoothACont();
		MotorState desB = smoothBCont();

		coActiveLeg->motorCurrentHip = pdH(desH.ang, rsActiveLeg->hip.legBodyAngle, desH.vel, rsActiveLeg->hip.legBodyVelocity);
		coActiveLeg->motorCurrentA   = pdA(desA.ang, rsActiveLeg->halfA.motorAngle, desA.vel, rsActiveLeg->halfA.motorVelocity);
		coActiveLeg->motorCurrentB   = pdB(desB.ang, rsActiveLeg->halfB.motorAngle, desB.vel, rsActiveLeg->halfB.motorVelocity);
	} else {
		// We're running now.
		coActiveLeg->motorCurrentHip = hipForce(rsActiveLeg->toeSwitch, rsActiveLeg->kneeForce,
			rsActiveLeg->hip.legBodyAngle, rsActiveLeg->hip.legBodyVelocity);

		double lockedDeflDiff = (flightLegPos.A - rsActiveLeg->halfA.legAngle) - (flightLegPos.B - rsActiveLeg->halfB.legAngle);

		LegState desState = forceCtrl(*rsActiveLeg, guiIn.force, 0.0);
		double forceDeflDiff  = (desState.A.ang - rsActiveLeg->halfA.legAngle) - (desState.B.ang - rsActiveLeg->halfB.legAngle);

		// Do force control only if it would cause a shortening in the virtual motor leg.
		if (lockedDeflDiff < forceDeflDiff) {
			desState.A.ang = (M_PI + lockedDeflDiff + rsActiveLeg->halfA.legAngle - rsActiveLeg->halfB.legAngle) / 2.0;
			desState.B.ang = (M_PI - lockedDeflDiff - rsActiveLeg->halfA.legAngle + rsActiveLeg->halfB.legAngle) / 2.0;
			desState.A.vel = 0.0;
			desState.B.vel = 0.0;
		} else {
			// Make sure it stays centered during force control.
			desState.A.ang = 0.5 * M_PI + (desState.A.ang - desState.B.ang) / 2.0;
			desState.B.ang = 0.5 * M_PI + (desState.B.ang - desState.A.ang) / 2.0;
			desState.A.vel = (desState.A.vel - desState.B.vel) / 2.0;
			desState.B.vel = (desState.B.vel - desState.A.vel) / 2.0;
		}
		coActiveLeg->motorCurrentA = pdA(desState.A.ang, rsActiveLeg->halfA.motorAngle, desState.A.vel, rsActiveLeg->halfA.rotorVelocity);
		coActiveLeg->motorCurrentB = pdB(desState.B.ang, rsActiveLeg->halfB.motorAngle, desState.B.vel, rsActiveLeg->halfB.rotorVelocity);

		// Add in feedforward torques to minimize steady-state error
		coActiveLeg->motorCurrentA += trqA(rsActiveLeg->halfA.motorAngle - rsActiveLeg->halfA.legAngle) / (50.0 * .121);
		coActiveLeg->motorCurrentB += trqB(rsActiveLeg->halfB.motorAngle - rsActiveLeg->halfB.legAngle) / (50.0 * .121);
	}

	// Let the GUI know the controller run state
	// Send data to the GUI
	if (pubTimer->readyToSend()) {
		guiOut.onGround = getOnGround();
		guiDataOut.write(guiOut);
	}

	co.command = medulla_state_run;

	// Check for issues
	if (fabs(coActiveLeg->motorCurrentHip) > 20.0)
	{
		co.command = medulla_state_error;
	}
	// Output for RTOps
	return co;
}

// Don't put control code below here!
bool ATCHipForceTest::configureHook() {
	RTT::TaskContext* hipForceInst = hipForceLoader.load(this, "asc_hip_force", "ASCHipForce");
	if (!hipForceInst)
		return false;
	flightP       = hipForceInst->properties()->getProperty("flightP");
	flightD       = hipForceInst->properties()->getProperty("flightD");
	stanceP       = hipForceInst->properties()->getProperty("stanceP");
	stanceD       = hipForceInst->properties()->getProperty("stanceD");
	toeFilterGain = hipForceInst->properties()->getProperty("toeFilterGain");
	toeThreshold  = hipForceInst->properties()->getProperty("toeThreshold");
	hipForce      = hipForceInst->provides("hipForce")->getOperation("runController");
	getOnGround   = hipForceInst->provides("hipForce")->getOperation("getOnGround");

	RTT::TaskContext* trqAInst = trqALoader.load(this, "asc_spring_torque", "ASCSpringTorque");
	if (!trqAInst)
		return false;
	trqA = trqAInst->provides("springTorque")->getOperation("getTorque");

	RTT::TaskContext* trqBInst = trqBLoader.load(this, "asc_spring_torque", "ASCSpringTorque");
	if (!trqBInst)
		return false;
	trqB = trqBInst->provides("springTorque")->getOperation("getTorque");

	RTT::TaskContext* pdHInst = pdHLoader.load(this, "asc_pd", "ASCPD");
	if (!pdHInst)
		return false;
	HP  = pdHInst->properties()->getProperty("P");
	HD  = pdHInst->properties()->getProperty("D");
	pdH = pdHInst->provides("pd")->getOperation("runController");

	RTT::TaskContext* pdAInst = pdALoader.load(this, "asc_pd", "ASCPD");
	if (!pdAInst)
		return false;
	AP  = pdAInst->properties()->getProperty("P");
	AD  = pdAInst->properties()->getProperty("D");
	pdA = pdAInst->provides("pd")->getOperation("runController");

	RTT::TaskContext* pdBInst = pdBLoader.load(this, "asc_pd", "ASCPD");
	if (!pdBInst)
		return false;
	BP  = pdBInst->properties()->getProperty("P");
	BD  = pdBInst->properties()->getProperty("D");
	pdB = pdBInst->provides("pd")->getOperation("runController");

	RTT::TaskContext* smoothHInst = smoothHLoader.load(this, "asc_path_generator", "ASCSmoothPathGenerator");
	if (!smoothHInst)
		return false;
	smoothHDone = smoothHInst->properties()->getProperty("isFinished");
	smoothHCont = smoothHInst->provides("smoothPath")->getOperation("runController");
	smoothHInit = smoothHInst->provides("smoothPath")->getOperation("init");

	RTT::TaskContext* smoothAInst = smoothALoader.load(this, "asc_path_generator", "ASCSmoothPathGenerator");
	if (!smoothAInst)
		return false;
	smoothADone = smoothAInst->properties()->getProperty("isFinished");
	smoothACont = smoothAInst->provides("smoothPath")->getOperation("runController");
	smoothAInit = smoothAInst->provides("smoothPath")->getOperation("init");

	RTT::TaskContext* smoothBInst = smoothBLoader.load(this, "asc_path_generator", "ASCSmoothPathGenerator");
	if (!smoothBInst)
		return false;
	smoothBDone = smoothBInst->properties()->getProperty("isFinished");
	smoothBCont = smoothBInst->provides("smoothPath")->getOperation("runController");
	smoothBInit = smoothBInst->provides("smoothPath")->getOperation("init");

	RTT::TaskContext* forceCtrlInst = forceCtrlLoader.load(this, "asc_force_control", "ASCForceControl");
	if (!forceCtrlInst)
		return false;
	forceCtrl = forceCtrlInst->provides("forceControl")->getOperation("getTgtState");

	legToMotorPos = this->provides("legToMotorTransforms")->getOperation("posTransform");

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
