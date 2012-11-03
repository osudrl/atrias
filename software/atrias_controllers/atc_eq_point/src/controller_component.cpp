/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_eq_point controller.
 */

#include <atc_eq_point/controller_component.h>

namespace atrias {
namespace controller {

ATCEqPoint::ATCEqPoint(std::string name) :
	RTT::TaskContext(name),
	logPort(name + "_log"),
	guiDataIn("gui_data_in")
{
	this->provides("atc")
	->addOperation("runController", &ATCEqPoint::runController, this, ClientThread)
	.doc("Get robot_state from RTOps and return controller output.");

	// Add properties
	this->addProperty("spg0Name", spg0Name);
	this->addProperty("spg1Name", spg1Name);
	this->addProperty("spg2Name", spg2Name);
	this->addProperty("spg3Name", spg3Name);
	this->addProperty("spg4Name", spg4Name);
	this->addProperty("spg5Name", spg5Name);
	this->addProperty("pd0Name",  pd0Name);
	this->addProperty("pd1Name",  pd1Name);
	this->addProperty("pd2Name",  pd2Name);
	this->addProperty("pd3Name",  pd3Name);
	this->addProperty("pd4Name",  pd4Name);
	this->addProperty("pd5Name",  pd5Name);

	// Gains for PD controllers. These are set in the configureHook.
	legP = 600;
	legD = 30;
	hipP = 0;
	hipD = 0;

	// For the GUI
	addEventPort(guiDataIn);
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

atrias_msgs::controller_output ATCEqPoint::runController(atrias_msgs::robot_state rs) {
	// Do nothing unless told otherwise
	co.lLeg.motorCurrentA   = 0.0;
	co.lLeg.motorCurrentB   = 0.0;
	co.lLeg.motorCurrentHip = 0.0;
	co.rLeg.motorCurrentA   = 0.0;
	co.rLeg.motorCurrentB   = 0.0;
	co.rLeg.motorCurrentHip = 0.0;

	MotorState desiredLAState,
	           desiredLBState,
	           desiredLHState,
	           desiredRAState,
	           desiredRBState,
	           desiredRHState;

	// Only run the controller when we're enabled
	if ((uint8_t)rs.cmState != (uint8_t) controllerManager::RtOpsCommand::ENABLE) {
		// Keep desired motor angles equal to the current motor angles so the
		// motors don't jump when the controller is enabled.
		desiredLAState.ang = rs.lLeg.halfA.motorAngle;
		desiredLBState.ang = rs.lLeg.halfB.motorAngle;
		desiredLHState.ang = rs.lLeg.hip.legBodyAngle;
		desiredRAState.ang = rs.rLeg.halfA.motorAngle;
		desiredRBState.ang = rs.rLeg.halfB.motorAngle;
		desiredRHState.ang = rs.rLeg.hip.legBodyAngle;

		// Reset smooth path generators. This is needed in case the user
		// disables the controller before disabling the demo, which is a
		// problem since the generators do not run when the controller is
		// disabled, which means they will finish only once the controller is
		// re-enabled, and that only with outdated data.
		spg0IsFinished = true;
		spg1IsFinished = true;
		spg2IsFinished = true;
		spg3IsFinished = true;
		spg4IsFinished = true;
		spg5IsFinished = true;

		// Set state to something other than 2.
		state = 0;

		return co;
	}

	// begin control code //

	// initial state
	// Calculate motor angles
	MotorAngle leftMotorAngle  = legToMotorPos(guiIn.pea, guiIn.l_leg_st);
	MotorAngle rightMotorAngle = legToMotorPos(guiIn.aea, guiIn.l_leg_st);
	int initDur = 2.0;   // Duration of initialization sequence.


	// Smoothly initialize and set state to 2.
	if (state == 0) {
		if (spg0IsFinished && spg1IsFinished && spg2IsFinished &&
		    spg3IsFinished && spg4IsFinished && spg5IsFinished) {
			spg0Init(rs.lLeg.halfA.motorAngle, leftMotorAngle.A, initDur);
			spg1Init(rs.lLeg.halfB.motorAngle, leftMotorAngle.B, initDur);
			spg2Init(rs.lLeg.hip.legBodyAngle, rs.lLeg.hip.legBodyAngle, initDur);
			spg3Init(rs.rLeg.halfA.motorAngle, rightMotorAngle.A, initDur);
			spg4Init(rs.rLeg.halfB.motorAngle, rightMotorAngle.B, initDur);
			spg5Init(rs.rLeg.hip.legBodyAngle, rs.rLeg.hip.legBodyAngle, initDur);

		}

		if (!spg0IsFinished && !spg1IsFinished && !spg2IsFinished &&
		    !spg3IsFinished && !spg4IsFinished && !spg5IsFinished) {
			desiredLAState = spg0RunController();
			desiredLBState = spg1RunController();
			desiredLHState = spg2RunController();
			desiredRAState = spg3RunController();
			desiredRBState = spg4RunController();
			desiredRHState = spg5RunController();

			co.lLeg.motorCurrentA   = pd0Controller(desiredLAState.ang, rs.lLeg.halfA.motorAngle, desiredLAState.vel, rs.lLeg.halfA.motorVelocity);
			co.lLeg.motorCurrentB   = pd1Controller(desiredLBState.ang, rs.lLeg.halfB.motorAngle, desiredLBState.vel, rs.lLeg.halfB.motorVelocity);
			co.lLeg.motorCurrentHip = pd2Controller(desiredLHState.ang, rs.lLeg.hip.legBodyAngle, desiredLHState.vel, rs.lLeg.hip.legBodyVelocity);
			co.rLeg.motorCurrentA   = pd3Controller(desiredRAState.ang, rs.rLeg.halfA.motorAngle, desiredRAState.vel, rs.rLeg.halfA.motorVelocity);
			co.rLeg.motorCurrentB   = pd4Controller(desiredRBState.ang, rs.rLeg.halfB.motorAngle, desiredRBState.vel, rs.rLeg.halfB.motorVelocity);
			co.rLeg.motorCurrentHip = pd5Controller(desiredRHState.ang, rs.rLeg.hip.legBodyAngle, desiredRHState.vel, rs.rLeg.hip.legBodyVelocity);

			if (spg0IsFinished && spg1IsFinished && spg2IsFinished &&
			    spg3IsFinished && spg4IsFinished && spg5IsFinished) {
				state = 2;
			}
		}

		return co;
	}

	// calculate leg angle and length

	l_rLeg = cos((rs.rLeg.halfA.motorAngle-rs.rLeg.halfB.motorAngle+2*M_PI)/2);
	phi_rLeg = (rs.rLeg.halfA.motorAngle+rs.rLeg.halfB.motorAngle)/2;
	l_lLeg = cos((rs.lLeg.halfA.motorAngle-rs.lLeg.halfB.motorAngle+2*M_PI)/2);
	phi_lLeg = (rs.lLeg.halfA.motorAngle+rs.lLeg.halfB.motorAngle)/2;


	// state machine
	if (state == 2 && phi_lLeg<M_PI/2 && rs.rLeg.toeSwitch)
	{
		state = 1;
		sw_stance = false;
	}
	if (state == 1 && phi_lLeg < M_PI/2 && rs.lLeg.toeSwitch)
	{
		state = 2;
		sw_stance = false;
	}

	// generate motor commands
	switch (state)          //stance leg right, flight leg left
	{
	case 1:
		if ((phi_rLeg < guiIn.pea) && !sw_stance) {           // stance leg rotate to pea
			switch (guiIn.control)
			{
				case 0:
				{
					co.rLeg.motorCurrentB = guiIn.l_st;                                     //constant stance leg current
					break;
				}
				case 1:
				{
					co.rLeg.motorCurrentB = guiIn.p_as * (guiIn.pea-phi_rLeg) + guiIn.d_as * (rs.rLeg.halfB.motorVelocity);                 // PD on leg angle
					break;
				}
				default:
					break;
			}

		} else { // if aea was reached once
			sw_stance=true;
			co.rLeg.motorCurrentB = 0;
		}
		co.rLeg.motorCurrentA = guiIn.p_ls * (guiIn.l_leg_st-l_rLeg) + guiIn.d_ls * (rs.rLeg.halfA.motorVelocity);              //keep leg length

		if ((phi_lLeg > guiIn.aea) && !sw_flight) {           // swing leg rotate to aea
			switch (guiIn.control)
			{
				case 0:
				{
					co.lLeg.motorCurrentA = guiIn.l_fl;                             //constant swing leg current
					break;
				}
				case 1:
				{
					co.lLeg.motorCurrentA = guiIn.p_ls * (guiIn.aea-phi_lLeg) + guiIn.d_af * (rs.lLeg.halfA.motorVelocity);
					break;
				}
				default:
					break;
			}
		} else { // aea is reached once
			sw_flight=true;
			co.lLeg.motorCurrentA = 0;
		}
		//map leg angle sweep of flight leg to 0->1
		s = (guiIn.pea-phi_lLeg) / (guiIn.pea - guiIn.aea);
		//keep desired leg length -> shorten leg depending on leg position
		l_swing = sin ( -M_PI/2 + 3*M_PI/2*s)*guiIn.l_leg_fl/2+guiIn.l_leg_st-guiIn.l_leg_fl/2;
		co.lLeg.motorCurrentB = guiIn.p_lf * (l_swing - l_lLeg) + guiIn.d_lf * (rs.lLeg.halfB.motorVelocity);

		break;

	case 2:                         // stance leg left, swing leg right
		if ((phi_lLeg < guiIn.pea) && !sw_stance) {           // stance leg rotate to pea
			switch (guiIn.control)
			{
				case 0:
				{
					co.lLeg.motorCurrentB = guiIn.l_st;                                     //constant stance leg current
					break;
				}
				case 1:
				{
					co.lLeg.motorCurrentB = guiIn.p_as * (guiIn.pea-phi_lLeg) + guiIn.d_as * (rs.lLeg.halfB.motorVelocity);                 // PD on leg angle
					break;
				}
				default:
					break;
			}
		} else {                        // if aea was reached once
			sw_stance=true;
			co.lLeg.motorCurrentB = 0;
		}
		co.lLeg.motorCurrentA = guiIn.p_ls * (guiIn.l_leg_st-l_lLeg) + guiIn.d_ls * (rs.lLeg.halfA.motorVelocity);              //keep leg length

		if ((phi_rLeg > guiIn.aea) && !sw_flight) {           // swing leg rotate to aea
			switch (guiIn.control)
			{
				case 0:
				{
					co.rLeg.motorCurrentA = guiIn.l_fl;                             //constant swing leg current
					break;
				}
				case 1:
				{
					co.rLeg.motorCurrentA = guiIn.p_ls * (guiIn.aea-phi_rLeg) + guiIn.d_af * (rs.rLeg.halfA.motorVelocity);
					break;
				}
				default:
					break;
			}
		} else { // aea is reached once
			sw_flight=true;
			co.rLeg.motorCurrentA = 0;
		}
		//map leg angle sweep of flight leg to 0->1
		s = (guiIn.pea-phi_rLeg) / (guiIn.pea - guiIn.aea);
		//keep desired leg length -> shorten leg depending on leg position
		l_swing = sin ( -M_PI/2 + 3*M_PI/2*s)*guiIn.l_leg_fl/2+guiIn.l_leg_st-guiIn.l_leg_fl/2;
		co.lLeg.motorCurrentB = guiIn.p_lf * (l_swing - l_rLeg) + guiIn.d_lf * (rs.rLeg.halfB.motorVelocity);

		break;

	default:
		break;
	} // end switch

	// Stuff the msg
	//co.lLeg.motorCurrentA = guiIn.des_motor_torque_A;
	//co.lLeg.motorCurrentB = guiIn.des_motor_torque_B;
	//co.lLeg.motorCurrentHip = guiIn.des_motor_torque_hip;

	// end control code //

	// Command a run state
	co.command = medulla_state_run;

	// Stuff the msg and push to ROS for logging
	logData.desiredState = 0.0;
	logPort.write(logData);

	// Output for RTOps
	return co;
}

// Don't put control code below here!
bool ATCEqPoint::configureHook() {
	// Connect to the subcontrollers
	// Get references to subcontroller component properties
	spg0 = this->getPeer(spg0Name);
	if (spg0) {
		spg0Init          = spg0->provides("smoothPath")->getOperation("init");
		spg0RunController = spg0->provides("smoothPath")->getOperation("runController");
		spg0IsFinished    = spg0->properties()->getProperty("isFinished");
	}

	spg1 = this->getPeer(spg1Name);
	if (spg1) {
		spg1Init          = spg1->provides("smoothPath")->getOperation("init");
		spg1RunController = spg1->provides("smoothPath")->getOperation("runController");
		spg1IsFinished    = spg1->properties()->getProperty("isFinished");
	}

	spg2 = this->getPeer(spg2Name);
	if (spg2) {
		spg2Init          = spg2->provides("smoothPath")->getOperation("init");
		spg2RunController = spg2->provides("smoothPath")->getOperation("runController");
		spg2IsFinished    = spg2->properties()->getProperty("isFinished");
	}

	spg3 = this->getPeer(spg3Name);
	if (spg3) {
		spg3Init          = spg3->provides("smoothPath")->getOperation("init");
		spg3RunController = spg3->provides("smoothPath")->getOperation("runController");
		spg3IsFinished    = spg3->properties()->getProperty("isFinished");
	}

	spg4 = this->getPeer(spg4Name);
	if (spg4) {
		spg4Init          = spg4->provides("smoothPath")->getOperation("init");
		spg4RunController = spg4->provides("smoothPath")->getOperation("runController");
		spg4IsFinished    = spg4->properties()->getProperty("isFinished");
	}

	spg5 = this->getPeer(spg5Name);
	if (spg5) {
		spg5Init          = spg5->provides("smoothPath")->getOperation("init");
		spg5RunController = spg5->provides("smoothPath")->getOperation("runController");
		spg5IsFinished    = spg5->properties()->getProperty("isFinished");
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

	P0.set(legP);
	D0.set(legD);
	P1.set(legP);
	D1.set(legD);
	P2.set(hipP);
	D2.set(hipD);
	P3.set(legP);
	D3.set(legD);
	P4.set(legP);
	D4.set(legD);
	P5.set(hipP);
	D5.set(hipD);


	legToMotorPos = this->provides("legToMotorTransforms")->getOperation("posTransform");

	log(Info) << "[ATCMT] configured!" << endlog();
	return true;
}

bool ATCEqPoint::startHook() {
	log(Info) << "[ATCMT] started!" << endlog();
	return true;
}

void ATCEqPoint::updateHook() {
	guiDataIn.read(guiIn);
}

void ATCEqPoint::stopHook() {
	log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCEqPoint::cleanupHook() {
	log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCEqPoint)

}
}
