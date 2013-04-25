/**
  * @file ATC_FORC_CONTROL_DEMO.hpp
  * @author Mikhail Jones
  * @brief This implements a demo force controller
  */

// TODO - Add flight and stance gains for both force and position control.

#include "atc_force_control_demo/ATCForceControlDemo.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// This constructor call is much simpler
ATCForceControlDemo::ATCForceControlDemo(string name) :
	ATC(name),
	ascCommonToolkit(this, "ascCommonToolkit"),
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
	// Initialize
	legRateLimit = 1.0;
	toePosition.left = 2.15;
	toePosition.right = 2.45;
	tL = tR = 0.0;
	duration = 2.5;
}

/* @brief This is the main function for the top-level controller
 * @param rs The robot state is an inherited member
 * @param co The controller output is an inhereted member
 */
void ATCForceControlDemo::controller() {
	
	/* Additionally, the following functions are available to command the robot state:
	 * commandHalt();    // Trigger a Halt
	 * commandEStop();   // Trigger an EStop
	 * commandDisable(); // Disable the robot
	 * setStartupEnabled(true/false) // Enable or disable the default startup controller
	 * setShutdownEnabled(true/false) // Enable or disable the shutdown controller
	 */
	
	// Startup is handled by the ATC class
	setStartupEnabled(true);
	
	// Update current robot state
    updateState();

	// Run hip controller
	hipController();

	// Left leg controller selection
	switch (lLegControllerState) {
		// Position control
		case 0:
			// Set motor angles
			std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(guiIn.left_leg_ang, guiIn.left_leg_len);

			// Rate limit motor velocities
			qLmA = ascRateLimitLmA(qLmA, legRateLimit);
			qLmB = ascRateLimitLmB(qLmB, legRateLimit);

			// Compute and set motor currents
			co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
			co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
			break;
			
		// Force control - constant
		case 1:
			// Get component forces
			fL.fx = guiIn.left_fx;
			fL.fz = guiIn.left_fz;
			fL.dfx = 0.0;
			fL.dfz = 0.0;
		
			// Compute and set motor current values
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForceLl.control(fL, rs.lLeg, rs.position);
			break;
			
		// Force control - sinewave
		case 2:
			// Compute forces
			fL.fx = guiIn.left_offx + guiIn.left_ampx*sin(tL*2.0*PI*guiIn.left_freqx);
			fL.fz = guiIn.left_offz + guiIn.left_ampz*sin(tL*2.0*PI*guiIn.left_freqz);
			fL.dfx = 2.0*PI*guiIn.left_ampx*guiIn.left_freqx*cos(tL*2.0*PI*guiIn.left_freqx);
			fL.dfz = 2.0*PI*guiIn.left_ampz*guiIn.left_freqz*cos(tL*2.0*PI*guiIn.left_freqz);
		
			// Compute and set motor current values
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForceLl.control(fL, rs.lLeg, rs.position);
			break;
			
		// Force control - automated sequence
		case 3:
			// Compute forces
			fL = automateForceSequence(tL);
		
			// Compute and set motor current values
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForceLl.control(fL, rs.lLeg, rs.position);		
			break;
			
	}
			
	// Right leg controller selection
	switch (rLegControllerState) {
		// Position control
		case 0:	
			// Set motor angles
			std::tie(qRmA, qRmB) = ascCommonToolkit.legPos2MotorPos(guiIn.right_leg_ang, guiIn.right_leg_len);

			// Rate limit motor velocities
			qRmA = ascRateLimitRmA(qRmA, legRateLimit);
			qRmB = ascRateLimitRmB(qRmB, legRateLimit);
			
			// Compute and set motor currents
			co.rLeg.motorCurrentA = ascPDLmA(qRmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
			co.rLeg.motorCurrentB = ascPDLmB(qRmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
			break;
			
		// Force control - constant
		case 1:
			// Get component forces
			fR.fx = guiIn.right_fx;
			fR.fz = guiIn.right_fz;
			fR.dfx = 0.0;
			fR.dfz = 0.0;
		
			// Compute and set motor current values
			std::tie(co.rLeg.motorCurrentA, co.rLeg.motorCurrentB) = ascLegForceRl.control(fR, rs.rLeg, rs.position);
			break;
			
		// Force control - sinewave
		case 2:
			// Compute forces
			fR.fx = guiIn.left_offx + guiIn.left_ampx*sin(tR*2.0*PI*guiIn.left_freqx);
			fR.fz = guiIn.left_offz + guiIn.left_ampz*sin(tR*2.0*PI*guiIn.left_freqz);
			fR.dfx = 2.0*PI*guiIn.left_ampx*guiIn.left_freqx*cos(tR*2.0*PI*guiIn.left_freqx);
			fR.dfz = 2.0*PI*guiIn.left_ampz*guiIn.left_freqz*cos(tR*2.0*PI*guiIn.left_freqz);;
		
			// Compute and set motor current values
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForceRl.control(fR, rs.lLeg, rs.position);
			break;
			
		// Force control - automated sequence
		case 3:
			// Compute forces
			fR = automateForceSequence(tR);
		
			// Compute and set motor current values
			std::tie(co.rLeg.motorCurrentA, co.rLeg.motorCurrentB) = ascLegForceRl.control(fR, rs.rLeg, rs.position);
			break;
			
	}

	// Copy over positions to the GUI output data
	guiOut.isEnabled = isEnabled();

	// Log data
	logOut.left_fx = fL.fx;
	logOut.left_fz = fL.fz;
	logOut.left_dfx = fL.dfx;
	logOut.left_dfz = fL.dfz;
	logOut.right_fx = fR.fx;
	logOut.right_fz = fR.fz;
	logOut.right_dfx = fR.dfx;
	logOut.right_dfz = fR.dfz;
	
	
}


// updateState
void ATCForceControlDemo::updateState() {

	// If we are disabled then reset rate limiters
	if (!isEnabled()) {
		ascRateLimitLmA.reset(rs.lLeg.halfA.motorAngle);
		ascRateLimitLmB.reset(rs.lLeg.halfB.motorAngle);
		ascRateLimitRmA.reset(rs.rLeg.halfA.motorAngle);
		ascRateLimitRmB.reset(rs.rLeg.halfB.motorAngle);
	}

	// Safety: If nothing else forces will be zero
	fL.fx = fR.fx = 0.0;
	fL.fz = fR.fz = 0.0;
	fL.dfx = fR.dfx = 0.0;
	fL.dfz = fR.dfz = 0.0;

	// Reset time count if controller is switched
	if (lLegControllerState != guiIn.left_controller) {
		tL = 0.0;
	} else {
		tL += 0.001;
	}
	if (rLegControllerState != guiIn.right_controller) {
		tR = 0.0;
	} else {
		tR += 0.001;
	}

	// Get GUI values
	lLegControllerState = guiIn.left_controller;
	rLegControllerState = guiIn.right_controller;	

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
	
	// Compute actual leg force from spring deflection
	fTemp = ascLegForceLl.compute(rs.lLeg, rs.position);
	fTemp = ascLegForceRl.compute(rs.rLeg, rs.position);

}


// hipController
void ATCForceControlDemo::hipController() {

	// Compute inverse kinematics
	std::tie(qLh, qRh) = ascHipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);
	
	// Compute and set motor currents
	co.lLeg.motorCurrentHip = ascPDLh(qLh, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentHip = ascPDRh(qRh, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);
        
}


// shutdownController
void ATCForceControlDemo::shutdownController() {

	// Compute and set motor currents (applies virtual dampers to all actuators)
	co.lLeg.motorCurrentA = ascPDLmA(0.0, 0.0, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDLmB(0.0, 0.0, 0.0, rs.lLeg.halfB.motorVelocity);
	co.lLeg.motorCurrentHip = ascPDLh(0.0, 0.0, 0.0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentA = ascPDRmA(0.0, 0.0, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDRmB(0.0, 0.0, 0.0, rs.rLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentHip = ascPDRh(0.0, 0.0, 0.0, rs.rLeg.hip.legBodyVelocity);

}


// automateForceSequence
LegForce ATCForceControlDemo::automateForceSequence(double t) {

	// Safety: If nothing else forces will be zero
	legForce.fx = 0.0; 
	legForce.fz = 0.0; 
	legForce.dfx = 0.0; 
	legForce.dfz = 0.0;

	// Piecewise force function
	if (t >= 0*duration && t < 1*duration) {
		legForce.fz = -5.0;
	
	} else if (t >= 1*duration && t < 2*duration) {
		legForce.fz = -100.0;
	
	} else if (t >= 2*duration && t < 3*duration) {
		legForce.fz = -200.0;
	
	} else if (t >= 3*duration && t < 4*duration) {
		legForce.fz = -300.0;
	
	} else if (t >= 4*duration && t < 5*duration) {
		legForce.fz = -400.0;
	
	} else if (t >= 5*duration && t < 6*duration) {
		legForce.fz = -500.0;
	
	} else if (t >= 6*duration && t < 7*duration) {
		legForce.fz = -400.0;
	
	} else if (t >= 7*duration && t < 8*duration) {
		legForce.fz = -300.0;
	
	} else if (t >= 8*duration && t < 9*duration) {
		legForce.fz = -200.0;
	
	} else if (t >= 9*duration && t < 10*duration) {
		legForce.fz = -100.0;
	
	} else if (t >= 10*duration && t < 11*duration) {
		legForce.fz = -5.0;
	
	} else if (t >= 11*duration && t < 12*duration) {
		legForce.fz = -250.0;
	
	} else if (t >= 12*duration && t < 13*duration) {
		legForce.fz = -500.0;
	
	} else if (t >= 13*duration && t < 14*duration) {
		legForce.fz = -250.0;
	
	} else if (t >= 14*duration && t < 15*duration) {
		legForce.fz = -5.0;
	
	} else if (t >= 15*duration && t < 16*duration) {
		legForce.fz = -500.0;
	
	} else if (t >= 16*duration && t < 17*duration) {
		legForce.fz = -5.0;
	
	}	
	
	// Return forces
	return legForce;

}


// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCForceControlDemo)

}
}
