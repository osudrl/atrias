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
	ascLegForceLl(this, "ascLegForceLl"),
	ascLegForceRl(this, "ascLegForceRl"),
	ascHipBoomKinematics(this, "ascHipBoomKinematics"),
	ascPDLmA(this, "ascPDLmA"),
	ascPDLmB(this, "ascPDLmB"),
	ascPDRmA(this, "ascPDRmA"),
	ascPDRmB(this, "ascPDRmB"),
	ascPDLh(this, "ascPDLh"),
	ascPDRh(this, "ascPDRh"),
	ascRateLimitLmA(this, "ascRateLimitLmA"),
	ascRateLimitLmB(this, "ascRateLimitLmB"),
	ascRateLimitRmA(this, "ascRateLimitRmA"),
	ascRateLimitRmB(this, "ascRateLimitRmB")
{
	// Set leg motor rate limit
	legRateLimit = 1.0;
	
	// Set hip controller toe positions
	toePosition.left = 2.15;
	toePosition.right = 2.45;
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
	hipController();

	// Main controller state machine
	switch (controllerState) {
		// Standing
		case 0:
			// Standing in place
			standingController();
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
			
		// Shutdown
		case 2:
			// Call shutdown controller
			shutdownController();
			break;
	}

	// Copy over positions to the GUI output data
	guiOut.isEnabled = isEnabled();
	 
}


void ATCSlipRunning::updateState() {

	// Get GUI values // TODO: clean up
	controllerState = guiIn.main_controller;
	stanceControlType = guiIn.stance_controller; // Passive and force
	ascSlipModel.r0 = guiIn.slip_leg;
	h = guiIn.hop_height;

	// Set leg motor position control PD gains
	ascPDLmA.P = ascPDLmB.P = ascPDRmA.P = ascPDRmB.P = guiIn.leg_pos_kp;
	ascPDLmA.D = ascPDLmB.D = ascPDRmA.D = ascPDRmB.D = guiIn.leg_pos_kd;

	// Set hip motors position control PD gains
	ascPDLh.P = ascPDRh.P = guiIn.hip_pos_kp;
	ascPDLh.D = ascPDRh.D = guiIn.hip_pos_kd;

	// Set leg motor force control PID gains
	ascLegForceLl.kp = ascLegForceRl.kp = guiIn.leg_for_kp;
	ascLegForceLl.ki = ascLegForceRl.ki = 0.0;
	ascLegForceLl.kd = ascLegForceRl.kd = guiIn.leg_for_kd;

}


void ATCSlipRunning::hipController() {

	// Compute inverse kinematics
	std::tie(qLh, qRh) = ascHipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);

	// Compute and set motor currents
	co.lLeg.motorCurrentHip = ascPDLh(qLh, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentHip = ascPDRh(qRh, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);
 
}


void ATCSlipRunning::standingController() {

	// Set leg angles
	qLl = qRl = PI/2.0;
	rLl = rRl = guiIn.standing_leg;

	// Compute motor angles
	std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(qLl, rLl);
	std::tie(qRmA, qRmB) = ascCommonToolkit.legPos2MotorPos(qRl, rRl);

	// Rate limit motor velocities
	qLmA = ascRateLimitLmA(qLmA, legRateLimit);
	qLmB = ascRateLimitLmB(qLmB, legRateLimit);
	qRmA = ascRateLimitRmA(qRmA, legRateLimit);
	qRmB = ascRateLimitRmB(qRmB, legRateLimit);

	// Compute and set motor currents
	co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentA = ascPDRmA(qRmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDRmB(qRmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
		
}


void ATCSlipRunning::shutdownController() {

	// Compute and set motor currents (applies virtual dampers to all actuators)
	co.lLeg.motorCurrentA = ascPDLmA(0.0, 0.0, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDLmB(0.0, 0.0, 0.0, rs.lLeg.halfB.motorVelocity);
	co.lLeg.motorCurrentHip = ascPDLh(0.0, 0.0, 0.0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentA = ascPDRmA(0.0, 0.0, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDRmB(0.0, 0.0, 0.0, rs.rLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentHip = ascPDRh(0.0, 0.0, 0.0, rs.rLeg.hip.legBodyVelocity);

}


void ATCSlipRunning::rightLegFlightFalling() {

// right leg is egb tracking angle while extending leg
// left leg mirror angle while shorten

}


void ATCSlipRunning::rightLegStance() {

// right leg force track
// left leg mirror angle and shorten or constant

}


void ATCSlipRunning::leftLegFlightRising() {

// left leg move from current angle to desired egb angle in calculated appex duration keep leg length constant or extend
// right leg mirror angle and keep constant length 

}


void ATCSlipRunning::leftLegFlightFalling() {

}


void ATCSlipRunning::leftLegStance() {

	// Spring type
	if (springType == 0) {
		ascSlipModel.k = ascCommonToolkit.legStiffness(slipState.r, ascSlipModel.r0);
	} else if (springType == 1) {
		ascSlipModel.k = guiIn.slip_spring;
	}

	// Compute SLIP force profile
	slipState = ascSlipModel.advanceRK5(slipState);
	legForce = ascSlipModel.force(slipState);

	// If SLIP model says we should be in flight...
	if (slipState.isFlight) {
		// Use last know leg position from stance
		co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
		co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);

	// If SLIP model says we should be in stance...
	} else {
		// Store last known leg position
		qLmA = rs.lLeg.halfA.legAngle;
		qLmB = rs.lLeg.halfB.legAngle;

		// Compute and set motor currents
		std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForceLl.control(legForce, rs.lLeg, rs.position);

	}

} else {
	// Set motor angles
	std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);

	// Compute and set motor currents
	co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);

}


void ATCSlipRunning::rightLegFlightRising() {

	// TODO - Intial conditions and linear interp
	// Redefine slip initial conditions incase we go into stance next time step
	slipState.r = ascSlipModel.r0;
	slipState.dr = rs.position.zVelocity;
	slipState.q = PI/2.0;
	slipState.dq = 0.0;

	// Set left leg motor angles
	std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0);
	
	// Set right leg motor angles
	std::tie(qRmA, qRmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0);

	// Rate limit motor velocities
	qLmA = ascRateLimitLmA(qLmA, legRateLimit);
	qLmB = ascRateLimitLmB(qLmB, legRateLimit);
	qRmA = ascRateLimitRmA(qRmA, legRateLimit);
	qRmB = ascRateLimitRmB(qRmB, legRateLimit);

	// Compute and set motor currents
	co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentA = ascPDRmA(qRmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDRmB(qRmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
}


// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCSlipRunning)

}
}
