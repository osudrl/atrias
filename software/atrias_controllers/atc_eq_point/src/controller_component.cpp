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
	
	// calculate leg angle and length

	l_rLeg = cos((rs.rLeg.halfA.motorAngle-rs.rLeg.halfB.motorAngle+2*pi)/2);
	phi_rLeg = (rs.rLeg.halfA.motorAngle+rs.rLeg.halfB.motorAngle)/2;
	l_lLeg = cos((rs.lLeg.halfA.motorAngle-rs.lLeg.halfB.motorAngle+2*pi)/2);
	phi_lLeg = (rs.lLeg.halfA.motorAngle+rs.lLeg.halfB.motorAngle)/2;
		
	
	// state machine
			
			if (state == 2 & phi_lLeg<pi/2 & rs.rLeg.toeSwitch = true)
			{
					state = 1;
					sw_stance = false;
			}
			if (state == 1 & phi_lLeg < pi/2 & rs.lLeg.toeSwitch = true)
			{
					state = 2
					sw_stance = false;
			}

	// generate motor commands
			switch (state)		//stance leg right, flight leg left
			{				
				case 1:
						if (phi_rLeg < guiIn.pea) & ~sw_stance	// stance leg rotate to PEA
						switch (guiIn.control_combobox)
							{
							case 0:				
								{
									co.rLeg.motorCurrentB = guiIn.l_st;			//constant stance leg current
								}
							case 1:
								{
									co.rLeg.motorCurrentB = guiIn.p_as * (guiIn.PEA-phi_rLeg) + guiIn.d_as * (rs.rLeg.halfB.motorVelocity); // PD on leg angle
								}
							}

						else		// if AEA was reached once
						{
								sw_stance=true;
								co.rLeg.motorCurrentB = 0;
						}
						co.rLeg.motorCurrentA = guiIn.p_ls*(guiIn.l_leg_st-l_rLeg)+guiIn.d_ls(rLeg.MotorA_velocity);	//keep leg length

						if (phi_lLeg > guiIn.aea) & ~sw_flight  // swing leg rotate to AEA
						
							switch (guiIn.control_combobox)
							{
							case 0:
								{
									co.lLeg.motorCurrentA = guiIn.l_fl;		//constant swing leg current
								}
							case 1:
								{
									co.lLeg.motorCurrentA = guiIn.p_ls * (guiIn.AEA-phi_lLeg) + guiIn.d_af * (rs.lLeg.halfA.motorVelocity);
								}
							}
						else		// AEA is reached once
						{
							sw_flight=true;
							co.lLeg.motorCurrentA = 0;
						}
						//map leg angle sweep of flight leg to 0->1
						s = (guiIn.pea-phi_lLeg) / (guiIn.pea - guiIn.AEA);
						//keep desired leg length -> shorten leg depending on leg position
						l_swing = sin ( -pi/2 + 3*pi/2*s)*guiIn.l_leg_fl/2+guiIn.l_leg_st-guiIn.l_leg_fl/2;
						co.lLeg.motorCurrentB = guiIn.p_lf * (l_swing - l_lLeg) + guiIn.d_lf * (rs.lLeg.halfB.motorVelocity);
						}
				case 2:			// stance leg left, swing leg right
						if (phi_lLeg < guiIn.pea) & ~sw_stance	// stance leg rotate to PEA
							switch (guiIn.control_combobox)
								{
								case 0:				
									{
										co.lLeg.motorCurrentB = guiIn.l_st;			//constant stance leg current
									}
								case 1:
									{
										co.lLeg.motorCurrentB = guiIn.p_as * (guiIn.PEA-phi_lLeg) + guiIn.d_as * (rs.lLeg.halfB.motorVelocity); // PD on leg angle
									}
								}

							else		// if AEA was reached once
							{
									sw_stance=true;
									co.lLeg.motorCurrentB = 0;
							}
							co.lLeg.motorCurrentA = guiIn.p_ls*(guiIn.l_leg_st-l_lLeg)+guiIn.d_ls(lLeg.MotorA_velocity);	//keep leg length

							if (phi_rLeg > guiIn.aea) & ~sw_flight  // swing leg rotate to AEA
						
								switch (guiIn.control_combobox)
								{
								case 0:
									{
										co.rLeg.motorCurrentA = guiIn.l_fl;		//constant swing leg current
									}
								case 1:
									{
										co.rLeg.motorCurrentA = guiIn.p_ls * (guiIn.AEA-phi_rLeg) + guiIn.d_af * (rs.rLeg.halfA.motorVelocity);
									}
								}
							else		// AEA is reached once
							{
								sw_flight=true;
								co.rLeg.motorCurrentA = 0;
							}
							//map leg angle sweep of flight leg to 0->1
							s = (guiIn.pea-phi_rLeg) / (guiIn.pea - guiIn.AEA);
							//keep desired leg length -> shorten leg depending on leg position
							l_swing = sin ( -pi/2 + 3*pi/2*s)*guiIn.l_leg_fl/2+guiIn.l_leg_st-guiIn.l_leg_fl/2;
							co.lLeg.motorCurrentB = guiIn.p_lf * (l_swing - l_rLeg) + guiIn.d_lf * (rs.rLeg.halfB.motorVelocity);
							}

			}
			
	
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
