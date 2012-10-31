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

    // Only run the controller when we're enabled
    if ((uint8_t)rs.cmState != (uint8_t)controllerManager::RtOpsCommand::ENABLE)
        return co;

    // begin control code //

	// initial state
	
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
