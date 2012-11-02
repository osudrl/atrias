/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_eq_point controller.
 */

#include <atc_eq_point/controller_component.h>

namespace atrias {
namespace controller {

ATCEqPoint::ATCEqPoint(std::string name):
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
    if ((uint8_t)rs.cmState != (uint8_t)controllerManager::RtOpsCommand::ENABLE) {
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

		return co;
	}

    // begin control code //

	// initial state
	// Calculate motor angles
	MotorAngle leftMotorAngle  = legToMotorPos(GUI.PEA, GUI.l_leg_stance);
	MotorAngle rightMotorAngle = legToMotorPos(GUI.AEA, GUI.l_leg_stance);
	int initDur = 2.0;   // Duration of initialization sequence.

	if (state != 2) {
		if (spg0IsFinished && spg1IsFinished && spg2IsFinished &&
				spg3IsFinished && spg4IsFinished && spg5IsFinished) {
			spg0Init(rs.lLeg.halfA.motorAngle, leftMotorAngle.A, initDur);
			spg1Init(rs.lLeg.halfB.motorAngle, leftMotorAngle.B, initDur);
			spg2Init(rs.lLeg.hip.legBodyAngle, rs.lLeg.hip.legBodyAngle, initDur);
			spg3Init(rs.rLeg.halfA.motorAngle, rightMotorAngle.A, initDur);
			spg4Init(rs.rLeg.halfB.motorAngle, rightMotorAngle.B, initDur);
			spg5Init(rs.rLeg.hip.legBodyAngle, rs.rLeg.hip.legBodyAngle, initDur);

		} else if (!spg0IsFinished && !spg1IsFinished && !spg2IsFinished &&
				!spg3IsFinished && !spg4IsFinished && !spg5IsFinished) {
			desiredLAState = spg0RunController();
			desiredLBState = spg1RunController();
			desiredLHState = spg2RunController();
			desiredRAState = spg3RunController();
			desiredRBState = spg4RunController();
			desiredRHState = spg5RunController();

			if (spg0IsFinished && spg1IsFinished && spg2IsFinished &&
					spg3IsFinished && spg4IsFinished && spg5IsFinished) {
				state = 2;
			}
		}

		return;
	}

		// move left leg to GUI.PEA and GUI.l_leg_stance
		// move right leg to GUI.AEA and GUI.l_leg_stance
		// (uint8_t)state=2
	
	// state machine
			if (state == 2 & rLeg.leg_angle<pi/2 & rLeg.toeswitch = true)
				{state = 1;}
				
			if (state == 1 & lLeg.leg_angle < pi/2 & lLeg.toeswitch = true)
				{state = 2}
				
	// 
			if (state == 1)						//stance leg right, flight leg left
				if control == constant			//apply constant current 
						if rleg.leg_angle < GUI.PEA
							{co.rLeg.motorCurrentB = l_st;
							 co.rLeg.motorCurrentA = GUI.p_ls*(GUI.l_leg_st-rLeg.leg_length)+GUI.d_ls(rLeg.MotorA_velocity)							
							}
							
			
	
    // Stuff the msg
    co.lLeg.motorCurrentA = guiIn.des_motor_torque_A;
    co.lLeg.motorCurrentB = guiIn.des_motor_torque_B;
    co.lLeg.motorCurrentHip = guiIn.des_motor_torque_hip;

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
