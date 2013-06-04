#include "atc_force_control_demo/ATCForceControlDemo.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// Controller constructor
ATCForceControlDemo::ATCForceControlDemo(string name) :
	ATC(name),
	ascCommonToolkit(this, "ascCommonToolkit"),
	ascHipBoomKinematics(this, "ascHipBoomKinematics"),
	ascLegForceL(this, "ascLegForceL"),
	ascLegForceR(this, "ascLegForceR"),
	ascPDLmA(this, "ascPDLmA"),
	ascPDLmB(this, "ascPDLmB"),
	ascPDRmA(this, "ascPDRmA"),
	ascPDRmB(this, "ascPDRmB"),
	ascPDLh(this, "ascPDLh"),
	ascPDRh(this, "ascPDRh"),
	ascRateLimitLmA(this, "ascRateLimitLmA"),
	ascRateLimitLmB(this, "ascRateLimitLmB"),
	ascRateLimitRmA(this, "ascRateLimitRmA"),
	ascRateLimitRmB(this, "ascRateLimitRmB"),
	ascRateLimitLh(this, "ascRateLimitLh"),
	ascRateLimitRh(this, "ascRateLimitRh")
{
	// Startup is handled by the ATC class
	setStartupEnabled(true);
	
	// Set leg motor rate limit
	legRateLimit = 0.2;
	hipRateLimit = 0.2;
	
	// Reset automated test counters
	tL = tR = 0.0;
}


void ATCForceControlDemo::controller() {	
	// Update current robot state
	updateState();

	// Run hip controller
	hipController();

	// Left leg controller state machine
	switch (lLegControllerState) {
		case 0: // Position control
		  // Set motor angles
		  std::tie(qmA, qmB) = ascCommonToolkit.legPos2MotorPos(guiIn.left_leg_ang, guiIn.left_leg_len);

		  // Rate limit motor velocities
		  qmA = ascRateLimitLmA(qmA, legRateLimit);
		  qmB = ascRateLimitLmB(qmB, legRateLimit);
		  dqmA = dqmB = 0.0;

		  // Compute and set motor currents
		  co.lLeg.motorCurrentA = ascPDLmA(qmA, rs.lLeg.halfA.motorAngle, dqmA, rs.lLeg.halfA.motorVelocity);
		  co.lLeg.motorCurrentB = ascPDLmB(qmB, rs.lLeg.halfB.motorAngle, dqmB, rs.lLeg.halfB.motorVelocity);
			break;
			
		case 1: // Force control - constant
			// Get component forces
			legForce.fx = guiIn.left_fx;
			legForce.fz = 0.0;
			legForce.dfx = 0.0;
			legForce.dfz = 0.0;
	
			// Compute and set motor current values
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForceL.control(legForce, rs.lLeg, rs.position);
			break;
			
		case 2: // Force control - sinewave
			// Run sinewave function
			legForce.fx = 0.0; legForce.dfx = 0.0;
			std::tie(legForce.fz, legForce.dfz) = sinewave(tL, guiIn.left_offz, guiIn.left_ampz, guiIn.left_freqz);
			
			// Compute and set motor current values
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForceL.control(legForce, rs.lLeg, rs.position);
			break;
		
		case 3: // Position control - automated stair step
			// Run stair step function		
			if ((tL >= 0) && (tL < 20)) {
				std::tie(rl, drl) = stairStep(tL, 0.90, -0.05, 20, 4);
			} else if ((tL >= 20) && (tL < 24)) {
				std::tie(rl, drl) = sinewaveSweep(tL - 20.0, 0.0, 2.0, 0.90, -0.05, 4);
			} else if ((tL >= 24) && (tL < 28)) {
				std::tie(rl, drl) = sinewaveSweep(tL - 24.0, 2.0, 0.0, 0.90, -0.05, 4);
			} else {
				rl = 0.90;
				drl = 0.0;
			}
			
		  // Set motor angles
		  std::tie(qmA, qmB) = ascCommonToolkit.legPos2MotorPos(M_PI/2.0, rl);
		  dqmA = 0.0; dqmB = 0.0;
		  
		  // Compute and set motor currents
		  co.lLeg.motorCurrentA = ascPDLmA(qmA, rs.lLeg.halfA.motorAngle, dqmA, rs.lLeg.halfA.motorVelocity);
		  co.lLeg.motorCurrentB = ascPDLmB(qmB, rs.lLeg.halfB.motorAngle, dqmB, rs.lLeg.halfB.motorVelocity);
			break;
			
		case 4: // Force control - automated stair step
			// Run stair step function
			legForce.fx = 0.0; legForce.dfx = 0.0;
			if ((tL >= 0) && (tL < 20)) {
				std::tie(legForce.fz, legForce.dfz) = stairStep(tL, 0, -400, 20, 4);
			} else {
				legForce.fz = 0.0;
				legForce.dfz = 0.0;
			}
				
			// Compute and set motor current values
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForceL.control(legForce, rs.lLeg, rs.position);
			break;		
	}
	
	// Right leg controller state machine
	switch (rLegControllerState) {
		case 0: // Position control
		  // Set motor angles
		  std::tie(qmA, qmB) = ascCommonToolkit.legPos2MotorPos(guiIn.right_leg_ang, guiIn.right_leg_len);

		  // Rate limit motor velocities
		  qmA = ascRateLimitRmA(qmA, legRateLimit);
		  qmB = ascRateLimitRmB(qmB, legRateLimit);
		  dqmA = dqmB = 0.0;

		  // Compute and set motor currents
		  co.rLeg.motorCurrentA = ascPDRmA(qmA, rs.rLeg.halfA.motorAngle, dqmA, rs.rLeg.halfA.motorVelocity);
		  co.rLeg.motorCurrentB = ascPDRmB(qmB, rs.rLeg.halfB.motorAngle, dqmB, rs.rLeg.halfB.motorVelocity);
			break;
			
		case 1: // Force control - constant
			// Get component forces
			legForce.fx = guiIn.right_fx;
			legForce.fz = 0.0;
			legForce.dfx = 0.0;
			legForce.dfz = 0.0;
	
			// Compute and set motor current values
			std::tie(co.rLeg.motorCurrentA, co.rLeg.motorCurrentB) = ascLegForceR.control(legForce, rs.rLeg, rs.position);
			break;
			
		case 2: // Force control - sinewave
			// Run sinewave function
			legForce.fx = 0.0; legForce.dfx = 0.0;
			std::tie(legForce.fz, legForce.dfz) = sinewave(tR, guiIn.right_offz, guiIn.right_ampz, guiIn.right_freqz);
			
			// Compute and set motor current values
			std::tie(co.rLeg.motorCurrentA, co.rLeg.motorCurrentB) = ascLegForceR.control(legForce, rs.rLeg, rs.position);
			break;
		
		case 3: // Position control - automated stair step
			// Run stair step function		
			if ((tR >= 0) && (tR < 20)) {
				std::tie(rl, drl) = stairStep(tR, 0.90, -0.05, 20, 4);
			} else if ((tR >= 20) && (tR < 24)) {
				std::tie(rl, drl) = sinewaveSweep(tR - 20.0, 0.0, 2.0, 0.90, -0.05, 4);
			} else if ((tR >= 24) && (tR < 28)) {
				std::tie(rl, drl) = sinewaveSweep(tR - 24.0, 2.0, 0.0, 0.90, -0.05, 4);
			} else {
				rl = 0.90;
				drl = 0.0;
			}
			
		  // Set motor angles
		  std::tie(qmA, qmB) = ascCommonToolkit.legPos2MotorPos(M_PI/2.0, rl);
		  dqmA = 0.0; dqmB = 0.0;
		  
		  // Compute and set motor currents
		  co.rLeg.motorCurrentA = ascPDRmA(qmA, rs.rLeg.halfA.motorAngle, dqmA, rs.rLeg.halfA.motorVelocity);
		  co.rLeg.motorCurrentB = ascPDRmB(qmB, rs.rLeg.halfB.motorAngle, dqmB, rs.rLeg.halfB.motorVelocity);
			break;
			
		case 4: // Force control - automated stair step
			// Run stair step function
			legForce.fx = 0.0; legForce.dfx = 0.0;
			if ((tR >= 0) && (tR < 20)) {
				std::tie(legForce.fz, legForce.dfz) = stairStep(tR, 0, -400, 20, 4);
			} else {
				legForce.fz = 0.0;
				legForce.dfz = 0.0;
			}
				
			// Compute and set motor current values
			std::tie(co.rLeg.motorCurrentA, co.rLeg.motorCurrentB) = ascLegForceR.control(legForce, rs.rLeg, rs.position);
			break;		
	}

	// Copy over positions to the GUI output data
	guiOut.isEnabled = isEnabled();
}


void ATCForceControlDemo::updateState() {
	// If we are disabled then reset all rate limiters
	if (!isEnabled()) {
		ascRateLimitLmA.reset(rs.lLeg.halfA.motorAngle);
		ascRateLimitLmB.reset(rs.lLeg.halfB.motorAngle);
		ascRateLimitRmA.reset(rs.rLeg.halfA.motorAngle);
		ascRateLimitRmB.reset(rs.rLeg.halfB.motorAngle);
		ascRateLimitLh.reset(rs.lLeg.hip.legBodyAngle);
		ascRateLimitRh.reset(rs.rLeg.hip.legBodyAngle);
	}

	// Reset time count and rate limiters if controller is switched
	if (lLegControllerState != guiIn.left_controller) {
		tL = 0.0;
		ascRateLimitLmA.reset(rs.lLeg.halfA.motorAngle);
		ascRateLimitLmB.reset(rs.lLeg.halfB.motorAngle);
	} else {
		tL += 0.001;
	}
	if (rLegControllerState != guiIn.right_controller) {
		tR = 0.0;
		ascRateLimitRmA.reset(rs.rLeg.halfA.motorAngle);
		ascRateLimitRmB.reset(rs.rLeg.halfB.motorAngle);
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
	ascLegForceL.kp = ascLegForceR.kp = guiIn.leg_for_kp;
	ascLegForceL.ki = ascLegForceR.ki = 0.0;
	ascLegForceL.kd = ascLegForceR.kd = guiIn.leg_for_kd;	

	// Compute actual leg force from spring deflection
	ascLegForceL.compute(rs.lLeg, rs.position);
	ascLegForceR.compute(rs.rLeg, rs.position);
}


void ATCForceControlDemo::hipController() {
	// Set hip controller toe positions
	toePosition.left = 2.15;//guiIn.left_toe_pos;
	toePosition.right = 2.45;//guiIn.right_toe_pos;
	
	// Compute inverse kinematics to keep lateral knee torque to a minimum
	std::tie(qLh, qRh) = ascHipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);

	// Rate limit motor velocities to smooth step inputs
	qLh = ascRateLimitLh(qLh, hipRateLimit);
	qRh = ascRateLimitRh(qRh, hipRateLimit);

	// Compute and set motor currents from position based PD controllers
	co.lLeg.motorCurrentHip = ascPDLh(qLh, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentHip = ascPDRh(qRh, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);
}


std::tuple<double, double> ATCForceControlDemo::sinewave(double t, double offset, double amplitude, double frequency) {
	// Compute forces
	y = offset + amplitude*sin(t*2.0*M_PI*frequency);
	dy = 2.0*M_PI*amplitude*frequency*cos(t*2.0*M_PI*frequency);
	
	// Return
	return std::make_tuple(y, dy);
}


std::tuple<double, double> ATCForceControlDemo::stairStep(double t, double offset, double amplitude, double omega, double steps) {
	// If nothing else forces will be zero
	y = 0.0; dy = 0.0;

	// Piecewise force function
	y = omega/2.0 - abs(fmod(t, omega) - omega/2.0);
	y = offset + amplitude*floor(2.0/omega*y*steps + 0.5)/steps;
		
	// Return
	return std::make_tuple(y, dy);
}


std::tuple<double, double> ATCForceControlDemo::sinewaveSweep(double t, double omega1, double omega2, double offset, double amplitude, double omega) {
	// Sinewave sweep
	a = M_PI*(omega2 - omega1)/(2.0*omega);
	b = 2.0*M_PI*omega1;
	y = offset + amplitude*sin(a*pow(t, 2) + b*t);
	dy = amplitude*cos(a*pow(t, 2) + b*t)*(b + 2.0*a*t);		

	// Return
	return std::make_tuple(y, dy);
}

// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCForceControlDemo)

}
}
