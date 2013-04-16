/**
  * @file ATC_FORC_CONTROL_DEMO.hpp
  * @author Mikhail Jones
  * @brief This implements a demo force controller.
  */
  
#include "atc_force_control_demo/ATCForceControlDemo.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// This constructor call is much simpler.
ATCForceControlDemo::ATCForceControlDemo(string name) :
	ATC(name),
	ascCommonToolkit(this, "ascCommonToolkit"),
	ascLegForce(this, "ascLegForce"),
	ascHipBoomKinematics(this, "ascHipBoomKinematics"),
	ascPDlA(this, "ascPDlA"),
	ascPDlB(this, "ascPDlB"),
	ascPDrA(this, "ascPDrA"),
	ascPDrB(this, "ascPDrB"),
	ascPDlh(this, "ascPDlh"),
	ascPDrh(this, "ascPDrh")
{
	// Initialize
	lt = rt = 0.0;
	duration = 6.0;
}

/* @brief This is the main function for the top-level controller.
 * @param rs The robot state is an inherited member.
 * @param co The controller output is an inhereted member.
 */
void ATCForceControlDemo::controller() {
	
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

	// Left leg controller selection
	switch (lLegControllerState) {
		// Position control
		case 0:
			// Set motor angles
			std::tie(qlmA, qlmB) = ascCommonToolkit.legPos2MotorPos(guiIn.left_leg_ang, guiIn.left_leg_len);

			// Compute and set motor currents
			co.lLeg.motorCurrentA = ascPDlA(qlmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
			co.lLeg.motorCurrentB = ascPDlB(qlmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
			break;
			
		// Force control - constant
		case 1:
			// Get component forces
			fl.fx = guiIn.left_fx;
			fl.fz = guiIn.left_fz;
			fl.dfx = 0.0;
			fl.dfz = 0.0;
		
			// Compute and set motor current values
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForce.control(fl, rs.lLeg, rs.position);
			
			// Compute and log computed leg forces
			tempLegForce = ascLegForce.compute(rs.lLeg, rs.position);
			break;
			
		// Force control - sinewave
		case 2:
			// Compute forces
			fl.fx = guiIn.left_offx + guiIn.left_ampx*sin(lt*2.0*PI*guiIn.left_freqx);
			fl.fz = guiIn.left_offz + guiIn.left_ampz*sin(lt*2.0*PI*guiIn.left_freqz);
			fl.dfx = 2.0*PI*guiIn.left_ampx*guiIn.left_freqx*cos(lt*2.0*PI*guiIn.left_freqx);
			fl.dfz = 2.0*PI*guiIn.left_ampz*guiIn.left_freqz*cos(lt*2.0*PI*guiIn.left_freqz);
		
			// Compute and set motor current values
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForce.control(fl, rs.lLeg, rs.position);
			
			// Compute and log computed leg forces
			tempLegForce = ascLegForce.compute(rs.lLeg, rs.position);
			break;
			
		// Force control - automated sequence
		case 3:
			// Compute forces
			fl = automateForceTest(lt);
		
			// Compute and set motor current values
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForce.control(fl, rs.lLeg, rs.position);
			
			// Compute and log computed leg forces
			tempLegForce = ascLegForce.compute(rs.lLeg, rs.position);		
			break;
			
	}
			
	// Right leg controller selection
	switch (rLegControllerState) {
		// Position control
		case 0:	
			// Set motor angles
			std::tie(qrmA, qrmB) = ascCommonToolkit.legPos2MotorPos(guiIn.right_leg_ang, guiIn.right_leg_len);

			// Compute and set motor currents
			co.rLeg.motorCurrentA = ascPDlA(qrmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
			co.rLeg.motorCurrentB = ascPDlB(qrmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
			break;
			
		// Force control - constant
		case 1:
			// Get component forces
			fr.fx = guiIn.right_fx;
			fr.fz = guiIn.right_fz;
			fr.dfx = 0.0;
			fr.dfz = 0.0;
		
			// Compute and set motor current values
			std::tie(co.rLeg.motorCurrentA, co.rLeg.motorCurrentB) = ascLegForce.control(fr, rs.rLeg, rs.position);
			
			// Compute and log computed leg forces
			tempLegForce = ascLegForce.compute(rs.rLeg, rs.position);
			break;
			
		// Force control - sinewave
		case 2:
			// Compute forces
			fr.fx = guiIn.left_offx + guiIn.left_ampx*sin(rt*2.0*PI*guiIn.left_freqx);
			fr.fz = guiIn.left_offz + guiIn.left_ampz*sin(rt*2.0*PI*guiIn.left_freqz);
			fr.dfx = 2.0*PI*guiIn.left_ampx*guiIn.left_freqx*cos(rt*2.0*PI*guiIn.left_freqx);
			fr.dfz = 2.0*PI*guiIn.left_ampz*guiIn.left_freqz*cos(rt*2.0*PI*guiIn.left_freqz);;
		
			// Compute and set motor current values
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForce.control(fr, rs.lLeg, rs.position);
			
			// Compute and log computed leg forces
			tempLegForce = ascLegForce.compute(rs.rLeg, rs.position);
			break;
			
		// Force control - automated sequence
		case 3:
			// Compute forces
			fr = automateForceTest(rt);
		
			// Compute and set motor current values
			std::tie(co.rLeg.motorCurrentA, co.rLeg.motorCurrentB) = ascLegForce.control(fr, rs.rLeg, rs.position);
			
			// Compute and log computed leg forces
			tempLegForce = ascLegForce.compute(rs.rLeg, rs.position);
			break;
			
	}

	// Copy over positions to the GUI output data
	guiOut.isEnabled = isEnabled();

	// Log data
	logOut.left_fx = fl.fx;
	logOut.left_fz = fl.fz;
	logOut.left_dfx = fl.dfx;
	logOut.left_dfz = fl.dfz;
	logOut.right_fx = fr.fx;
	logOut.right_fz = fr.fz;
	logOut.right_dfx = fr.dfx;
	logOut.right_dfz = fr.dfz;
	
	
}


// updateState
void ATCForceControlDemo::updateState() {

	// Safety: If nothing else forces will be zero.
	fl.fx = fr.fx = 0.0;
	fl.fz = fr.fz = 0.0;
	fl.dfx = fr.dfx = 0.0;
	fl.dfz = fr.dfz = 0.0;

	// Reset time count if controller is switched
	if (lLegControllerState != guiIn.left_controller) {
		lt = 0.0;
	} else {
		lt += 0.001;
	}
	if (rLegControllerState != guiIn.right_controller) {
		rt = 0.0;
	} else {
		rt += 0.001;
	}

	// Get GUI values
	lLegControllerState = guiIn.left_controller;
	rLegControllerState = guiIn.right_controller;	

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
void ATCForceControlDemo::hipControl() {

	// Set toe positions
	toePosition.left = 2.15;
	toePosition.right = 2.45;

	// Compute inverse kinematics
	std::tie(qlh, qrh) = ascHipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);
	
	// Compute and set motor currents
	co.lLeg.motorCurrentHip = ascPDlh(qlh, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentHip = ascPDrh(qrh, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);
        
}


// automateForceTest
LegForce ATCForceControlDemo::automateForceTest(double t) {

	// Safety: If nothing else forces will be zero.
	legForce.fx = 0.0; 
	legForce.fz = 0.0; 
	legForce.dfx = 0.0; 
	legForce.dfz = 0.0;

	// Piecewise force function
	if (t >= 0*duration && t < 1*duration) {
		legForce.fz = 0.0;
	
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
		legForce.fz = 0.0;
	
	} else if (t >= 11*duration && t < 12*duration) {
		legForce.fz = -250.0;
	
	} else if (t >= 12*duration && t < 13*duration) {
		legForce.fz = -500.0;
	
	} else if (t >= 13*duration && t < 14*duration) {
		legForce.fz = -250.0;
	
	} else if (t >= 14*duration && t < 15*duration) {
		legForce.fz = 0.0;
	
	} else if (t >= 15*duration && t < 16*duration) {
		legForce.fz = -500.0;
	
	} else if (t >= 16*duration && t < 17*duration) {
		legForce.fz = 0.0;
	
	}	
	
	// Return forces
	return legForce;

}


// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCForceControlDemo)

}
}
