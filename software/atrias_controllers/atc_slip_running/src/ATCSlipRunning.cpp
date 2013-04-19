/**
  * @file ATC_SLIP_RUNNING.cpp
  * @author Mikhail Jones
  * @brief This implements a SLIP based template controller.
  */
  
#include "atc_slip_running/ATCSlipRunning.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// This constructor call is much simpler.
ATCSlipRunning::ATCSlipRunning(string name) :
	ATC(name),
	ascCommonToolkit(this, "ascCommonToolkit"),
	ascSlipModel(this, "ascSlipModel"),
	ascLegForce(this, "ascLegForce"),
	ascHipBoomKinematics(this, "ascHipBoomKinematics"),
	ascPDlA(this, "ascPDlA"),
	ascPDlB(this, "ascPDlB"),
	ascPDrA(this, "ascPDrA"),
	ascPDrB(this, "ascPDrB"),
	ascPDlh(this, "ascPDlh"),
	ascPDrh(this, "ascPDrh")
{
	// Nothing to see here.
}

/* @brief This is the main function for the top-level controller.
 * @param rs The robot state is an inherited member.
 * @param co The controller output is an inhereted member.
 */
void ATCSlipRunning::controller() {
	
	/* Additionally, the following functions are available to command the robot state:
	 * commandHalt();    // Trigger a Halt
	 * commandEStop();   // Trigger an EStop
	 * commandDisable(); // Disable the robot
	 * setStartupEnabled(true/false) // Enable or disable the default startup controller
	 * setShutdownEnabled(true/false) // Enable or disable the shutdown controller
	 */
	 
    // Startup is handled by the ATC class.
    setStartupEnabled(true);
    
    // Update current robot state
    updateState();

	// Run hip controller
	hipControl();

	// Main controller state machine
	switch (controllerState) {
		// Standing
		case 0:
			// Standing in place
			standingControl();
			break;
		
		// SLIP running
		case 1:
			// SLIP running controller state machine
			switch (runningState) {
				// Right leg flight - falling
				case 0:
					rightLegFlightFalling();
					if (rs.position.zPosition < 0.85) {
						runningState = 1;
					}					
					break;
				
				// Right leg stance
				case 1:
					rightLegStance();
					if (rs.position.zPosition > 0.85) {
						runningState = 2;
					}
					break;
					
				// Left leg flight - rising
				case 2:
					leftLegFlightRising();
					if (rs.position.zVelocity < 0.0) {
						runningState = 3;
					}
					break;
					
				// Left leg flight - falling
				case 3:
					leftLegFlightFalling();
					break;
					
				// Left leg stance
				case 4:
					leftLegStance();
					break;
				
				// Right leg flight - rising
				case 5:
					rightLegFlightRising();
					break;
					
			}
			break;
	}

	// Copy over positions to the GUI output data
	guiOut.isEnabled = isEnabled();
	 
}


// updateState
void ATCSlipRunning::updateState() {

	// Get GUI values
	controllerState = guiIn.main_controller;
	runningType = guiIn.hop_type;
	ascSlipModel.r0 = guiIn.slip_leg;
	h = guiIn.hop_height;

	// Set leg motor position control PD gains
	ascPDlA.P = ascPDlB.P = ascPDrA.P = ascPDrB.P = guiIn.leg_pos_kp;
	ascPDlA.D = ascPDlB.D = ascPDrA.D = ascPDrB.D = guiIn.leg_pos_kd;
	
	// Set hip motors position control PD gains
	ascPDlh.P = ascPDrh.P = guiIn.hip_pos_kp;
	ascPDlh.D = ascPDrh.D = guiIn.hip_pos_kd;
	
	// Set leg motor force control PID gains
	ascLegForce.kp = guiIn.leg_for_kp;
	ascLegForce.ki = 0.0;
	ascLegForce.kd = guiIn.leg_for_kd;

}


// hipControl
void ATCSlipRunning::hipControl() {

	// Set toe positions
	toePosition.left = 2.15;
	toePosition.right = 2.45;

	// Compute inverse kinematics
	std::tie(qlh, qrh) = ascHipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);
	
	// Compute and set motor currents
	co.lLeg.motorCurrentHip = ascPDlh(qlh, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentHip = ascPDrh(qrh, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);
        
}


// standingControl
void ATCSlipRunning::standingControl() {
	
	// Set leg angles
	qll = qrl = PI/2.0;
	rll = rrl = guiIn.standing_leg;
	
	// Compute motor angles
	std::tie(qlmA, qlmB) = ascCommonToolkit.legPos2MotorPos(qll, rrl);
	std::tie(qrmA, qrmB) = ascCommonToolkit.legPos2MotorPos(qll, rrl);
	
	// Compute and set motor currents
	co.lLeg.motorCurrentA = ascPDlA(qlmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDlB(qlmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentA = ascPDrA(qrmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDrB(qrmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
		
}


// rightLegFlightFalling
void ATCSlipRunning::rightLegFlightFalling() {

// right leg is egb tracking angle while extending leg
// left leg mirror angle while shorten

}


// rightLegStance
void ATCSlipRunning::rightLegStance() {

// right leg force track
// left leg mirror angle and shorten or constant

}


// leftLegFlightRising
void ATCSlipRunning::leftLegFlightRising() {

// left leg move from current angle to desired egb angle in calculated appex duration keep leg length constant or extend
// right leg mirror angle and keep constant length 

}


// leftLegFlightFalling
void ATCSlipRunning::leftLegFlightFalling() {

}


// leftLegStance
void ATCSlipRunning::leftLegStance() {

	// Spring
	ascSlipModel.k = ascCommonToolkit.legStiffness(slipState.r, ascSlipModel.r0);

	// Compute SLIP force profile
	slipState = ascSlipModel.advanceRK5(slipState);
	legForce = ascSlipModel.force(slipState);

	// If SLIP model says we should be in flight...
	if (slipState.isFlight) {
		// Use last know leg position from stance
		co.lLeg.motorCurrentA = ascPDlA(qlmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
		co.lLeg.motorCurrentB = ascPDlB(qlmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	
	// If SLIP model says we should be in stance...
	} else {				
		// Store last known leg position
		qlmA = rs.lLeg.halfA.legAngle;
		qlmB = rs.lLeg.halfB.legAngle;

		// Compute and set motor currents
		std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForce.control(legForce, rs.lLeg, rs.position);
	}

	// Set motor angles
	std::tie(qrmA, qrmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.75);

	// Compute and set motor currents
	co.rLeg.motorCurrentA = ascPDrA(qrmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDrB(qrmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);

}


// rightLegFlightRising
void ATCSlipRunning::rightLegFlightRising() {

	// Redefine slip initial conditions incase we go into stance next time step
	slipState.r = ascSlipModel.r0;
	slipState.dr = rs.position.zVelocity;
	slipState.q = PI/2.0;
	slipState.dq = 0.0;

	// Set left leg motor angles
	std::tie(qlmA, qlmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0);
	
	// Set right leg motor angles
	std::tie(qrmA, qrmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0);

	// Compute and set motor currents
	co.lLeg.motorCurrentA = ascPDlA(qlmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDlB(qlmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentA = ascPDrA(qrmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDrB(qrmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
}


// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCSlipRunning)

}
}
