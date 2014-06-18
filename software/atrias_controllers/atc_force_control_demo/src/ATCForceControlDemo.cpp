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
	legRateLimit = 0.5; // [rad/s]
	hipRateLimit = 0.5; // [rad/s]
	
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
			legForce.fz = guiIn.left_fz;
			legForce.dfx = 0.0;
			legForce.dfz = 0.0;

			// Compute and set motor current values
			//std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForceL.control(legForce, rs.lLeg, rs.position);
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = legForceControl(legForce, rs.lLeg, rs.position);
			break;

		case 2: // Force control - sinewave
			// Run sinewave function
			legForce.fx = 0.0; legForce.dfx = 0.0;
			std::tie(legForce.fz, legForce.dfz) = sinewave(tL, guiIn.left_offz, guiIn.left_ampz, guiIn.left_freqz);

			// Compute and set motor current values
			//std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForceL.control(legForce, rs.lLeg, rs.position);
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = legForceControl(legForce, rs.lLeg, rs.position);
			break;

		case 3: // Position control - automated stair step
			// Run stair step function		
			if ((tL >= 0) && (tL < 20)) {
				std::tie(rl, drl) = stairStep(tL, guiIn.left_leg_len, -0.05, 20, 4);
			} else if ((tL >= 20) && (tL < 24)) {
				std::tie(rl, drl) = sinewaveSweep(tL - 20.0, 0.0, 2.0, guiIn.left_leg_len, -0.05, 4);
			} else if ((tL >= 24) && (tL < 28)) {
				std::tie(rl, drl) = sinewaveSweep(tL - 24.0, 2.0, 0.0, guiIn.left_leg_len, -0.05, 4);
			} else {
				tL = 0.0;
				rl = guiIn.left_leg_len;
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
				std::tie(legForce.fz, legForce.dfz) = stairStep(tL, -20.0, -400, 20, 4);
			} else {
				legForce.fz = -20.0;
				legForce.dfz = 0.0;
			}

			// Compute and set motor current values
			//std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForceL.control(legForce, rs.lLeg, rs.position);
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = legForceControl(legForce, rs.lLeg, rs.position);
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
			legForce.fz = guiIn.right_fz;
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
				std::tie(rl, drl) = stairStep(tR, guiIn.right_leg_len, -0.05, 20, 4);
			} else if ((tR >= 20) && (tR < 24)) {
				std::tie(rl, drl) = sinewaveSweep(tR - 20.0, 0.0, 2.0, guiIn.right_leg_len, -0.05, 4);
			} else if ((tR >= 24) && (tR < 28)) {
				std::tie(rl, drl) = sinewaveSweep(tR - 24.0, 2.0, 0.0, guiIn.right_leg_len, -0.05, 4);
			} else {
				tR = 0.0;
				rl = guiIn.right_leg_len;
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

	// Set feedback linearization force control gains
	k1_11 = k1_22 = guiIn.leg_for_kd;
	k2_11 = k2_22 = k1_11*k1_11;

	// Compute actual leg force from spring deflection
	ascLegForceL.compute(rs.lLeg, rs.position);
	ascLegForceR.compute(rs.rLeg, rs.position);
}


void ATCForceControlDemo::hipController() {
	// Set hip controller toe positions
	toePosition.left = 2.17;
	toePosition.right = 2.5;
	
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

std::tuple<double, double> ATCForceControlDemo::legForceControl(LegForce legForce, atrias_msgs::robot_state_leg leg, atrias_msgs::robot_state_location position) {
    // Unpack parameters
    double FxDes = -legForce.fx;
    double FyDes = -legForce.fz;
    double qlA   = leg.halfA.legAngle;
    double qlB   = leg.halfB.legAngle;
    double qmA   = leg.halfA.motorAngle;
    double qmB   = leg.halfB.motorAngle;
    double qb    = position.bodyPitch;
    double dqlA  = leg.halfA.legVelocity;
    double dqlB  = leg.halfB.legVelocity;
    double dqmA  = leg.halfA.motorVelocity;
    double dqmB  = leg.halfB.motorVelocity;
    double dqb   = position.bodyPitchVelocity;

    // Remove torso tilt (Convert to world coordinates)
    qlA  += qb - 3.0*M_PI/2.0;
    qlB  += qb - 3.0*M_PI/2.0;
    qmA  += qb - 3.0*M_PI/2.0;
    qmB  += qb - 3.0*M_PI/2.0;
    dqlA += dqb;
    dqlB += dqb;
    dqmA += dqb;
    dqmB += dqb;

    // Convert to simulation coordinates
    // Angles (rad)
    double q1 = M_PI - qlB;
    double q2 = qlB - qlA;
    double q3 = M_PI - qmA;
    double q6 = M_PI - qmB;
    // Angular velocities (rad/s)
    double dq1 = -dqlB;
    double dq2 = dqlB - dqlA;
    double dq3 = -dqmA;
    double dq6 = -dqmB;

    // Constants and ATRIAS parameters
    const double g  = G;       // Gravity (m/s^2)
    const double r1 = L1;      // Length of link 1 (m)
    const double r2 = L2;      // Length of link 2
    const double m  = M;       // Mass of ATRIAS (kg)
    const double I  = 0.0019;  // Rotor inertia (kg*m^2)
    const double I3 = I*KG*KG; // Rotor inertia (as seen by the output)
    const double I6 = I3;      // Rotor inertia
    const double c3 = 19.0;    // Motor damping (as seen by the output) (N*m*s/rad)
    const double c6 = c3;      // Motor damping
    const double ks = KS;      // Spring constant (N/rad)
    const double cS = 1.49;    // Spring damping (N*m*s/rad)

    // Compute the desired torque using feedback linearization.  Used MATLAB
    // for the derivation; code available here:
    // https://github.com/andrewPeekema/atriasLeg
    double tauA = (1.0/(r2*r2)*(I3*(ks*ks)*q1*r1*-4.0-I3*(ks*ks)*q2*r1*4.0+I3*(ks*ks)*q3*r1*4.0+I3*(ks*ks)*q1*r2*cos(q2)*4.0-I3*(ks*ks)*q6*r2*cos(q2)*4.0-(ks*ks)*m*q1*r1*(r2*r2)*2.0-(ks*ks)*m*q2*r1*(r2*r2)*2.0+(ks*ks)*m*q3*r1*(r2*r2)*2.0-I3*cS*dq1*ks*r1*4.0-I3*cS*dq2*ks*r1*4.0+I3*cS*dq3*ks*r1*4.0+FxDes*I3*g*m*r1*(r2*r2)+I3*cS*dq1*ks*r2*cos(q2)*4.0-I3*cS*dq6*ks*r2*cos(q2)*4.0+c3*dq3*ks*m*r1*(r2*r2)*2.0-cS*dq1*ks*m*r1*(r2*r2)*2.0-cS*dq2*ks*m*r1*(r2*r2)*2.0+cS*dq3*ks*m*r1*(r2*r2)*2.0-FxDes*I3*cS*dq1*(r2*r2)*cos(q1)*2.0+FxDes*I3*cS*dq6*(r2*r2)*cos(q1)*2.0-FyDes*I3*cS*dq1*(r2*r2)*sin(q1)*2.0+FyDes*I3*cS*dq6*(r2*r2)*sin(q1)*2.0-FxDes*I3*ks*q1*(r2*r2)*cos(q1)*2.0+FxDes*I3*ks*q6*(r2*r2)*cos(q1)*2.0+(ks*ks)*m*q1*r1*(r2*r2)*cos(q2*2.0)*2.0+(ks*ks)*m*q2*r1*(r2*r2)*cos(q2*2.0)*2.0-(ks*ks)*m*q3*r1*(r2*r2)*cos(q2*2.0)*2.0-FyDes*I3*ks*q1*(r2*r2)*sin(q1)*2.0+FyDes*I3*ks*q6*(r2*r2)*sin(q1)*2.0-FxDes*I3*cS*dq1*(r2*r2)*cos(q1+q2*2.0)*2.0+FxDes*I3*cS*dq6*(r2*r2)*cos(q1+q2*2.0)*2.0-FyDes*I3*cS*dq1*(r2*r2)*sin(q1+q2*2.0)*2.0+FyDes*I3*cS*dq6*(r2*r2)*sin(q1+q2*2.0)*2.0-FxDes*I3*ks*q1*(r2*r2)*cos(q1+q2*2.0)*2.0+FxDes*I3*ks*q6*(r2*r2)*cos(q1+q2*2.0)*2.0-FyDes*I3*ks*q1*(r2*r2)*sin(q1+q2*2.0)*2.0+FyDes*I3*ks*q6*(r2*r2)*sin(q1+q2*2.0)*2.0-FyDes*I3*(dq1*dq1)*m*r1*(r2*r2*r2)*cos(q1+q2)*2.0-FyDes*I3*(dq2*dq2)*m*r1*(r2*r2*r2)*cos(q1+q2)*2.0-FyDes*I3*k2_11*m*r1*(r2*r2*r2)*cos(q1-q2)-FyDes*I3*k2_11*m*r1*(r2*r2*r2)*cos(q1+q2*3.0)+FxDes*I3*(dq1*dq1)*m*r1*(r2*r2*r2)*sin(q1+q2)*2.0+FxDes*I3*(dq2*dq2)*m*r1*(r2*r2*r2)*sin(q1+q2)*2.0+FxDes*I3*k2_11*m*r1*(r2*r2*r2)*sin(q1-q2)+FxDes*I3*k2_11*m*r1*(r2*r2*r2)*sin(q1+q2*3.0)-FxDes*I3*g*m*r1*(r2*r2)*cos(q1*2.0)-FxDes*I3*g*m*r1*(r2*r2)*cos(q2*2.0)+FxDes*I3*cS*dq1*r1*r2*cos(q1+q2)*4.0+FxDes*I3*cS*dq2*r1*r2*cos(q1+q2)*4.0-FxDes*I3*cS*dq3*r1*r2*cos(q1+q2)*4.0-FyDes*I3*g*m*r1*(r2*r2)*sin(q1*2.0)-FyDes*I3*g*m*r1*(r2*r2)*sin(q2*2.0)+FyDes*I3*cS*dq1*r1*r2*sin(q1+q2)*4.0+FyDes*I3*cS*dq2*r1*r2*sin(q1+q2)*4.0-FyDes*I3*cS*dq3*r1*r2*sin(q1+q2)*4.0+FxDes*I3*ks*q1*r1*r2*cos(q1+q2)*4.0+FxDes*I3*ks*q2*r1*r2*cos(q1+q2)*4.0-FxDes*I3*ks*q3*r1*r2*cos(q1+q2)*4.0+FyDes*I3*ks*q1*r1*r2*sin(q1+q2)*4.0+FyDes*I3*ks*q2*r1*r2*sin(q1+q2)*4.0-FyDes*I3*ks*q3*r1*r2*sin(q1+q2)*4.0+FyDes*I3*(dq1*dq1)*m*r1*(r2*r2*r2)*cos(q1-q2)*2.0+FyDes*I3*(dq2*dq2)*m*r1*(r2*r2*r2)*cos(q1-q2)*2.0+FxDes*I3*g*m*r1*(r2*r2)*cos(q1*2.0+q2*2.0)-I3*(dq1*dq1)*ks*m*(r1*r1)*r2*sin(q2)*4.0-I3*g*ks*m*r1*r2*cos(q1+q2)*2.0-c3*dq3*ks*m*r1*(r2*r2)*cos(q2*2.0)*2.0+cS*dq1*ks*m*r1*(r2*r2)*cos(q2*2.0)*2.0+cS*dq2*ks*m*r1*(r2*r2)*cos(q2*2.0)*2.0-cS*dq3*ks*m*r1*(r2*r2)*cos(q2*2.0)*2.0-FxDes*I3*(dq1*dq1)*m*r1*(r2*r2*r2)*sin(q1-q2)*2.0-FxDes*I3*(dq2*dq2)*m*r1*(r2*r2*r2)*sin(q1-q2)*2.0+FyDes*I3*g*m*r1*(r2*r2)*sin(q1*2.0+q2*2.0)+I3*dq1*k1_11*ks*m*r1*(r2*r2)*2.0+I3*dq2*k1_11*ks*m*r1*(r2*r2)*2.0-I3*dq3*k1_11*ks*m*r1*(r2*r2)*2.0+FyDes*I3*(dq1*dq1)*m*(r1*r1)*(r2*r2)*cos(q1)*2.0+I3*k2_11*ks*m*q1*r1*(r2*r2)*2.0+I3*k2_11*ks*m*q2*r1*(r2*r2)*2.0-I3*k2_11*ks*m*q3*r1*(r2*r2)*2.0-FxDes*I3*(dq1*dq1)*m*(r1*r1)*(r2*r2)*sin(q1)*2.0+FyDes*I3*k2_11*m*r1*(r2*r2*r2)*cos(q1+q2)*2.0-FxDes*I3*k2_11*m*r1*(r2*r2*r2)*sin(q1+q2)*2.0-FyDes*I3*(dq1*dq1)*m*(r1*r1)*(r2*r2)*cos(q1+q2*2.0)*2.0-I3*(dq1*dq1)*ks*m*r1*(r2*r2)*sin(q2*2.0)*2.0-I3*(dq2*dq2)*ks*m*r1*(r2*r2)*sin(q2*2.0)*2.0+I3*g*ks*m*r1*r2*cos(q1-q2)*2.0+FxDes*I3*(dq1*dq1)*m*(r1*r1)*(r2*r2)*sin(q1+q2*2.0)*2.0-FyDes*I3*dq1*dq2*m*r1*(r2*r2*r2)*cos(q1+q2)*4.0-FxDes*I3*dq1*k1_11*m*r1*(r2*r2*r2)*cos(q1+q2)*2.0-FxDes*I3*dq2*k1_11*m*r1*(r2*r2*r2)*cos(q1+q2)*2.0+FxDes*I3*dq1*dq2*m*r1*(r2*r2*r2)*sin(q1+q2)*4.0-FyDes*I3*dq1*k1_11*m*r1*(r2*r2*r2)*sin(q1+q2)*2.0-FyDes*I3*dq2*k1_11*m*r1*(r2*r2*r2)*sin(q1+q2)*2.0+FyDes*I3*dq1*dq2*m*r1*(r2*r2*r2)*cos(q1-q2)*4.0+FxDes*I3*dq1*k1_11*m*r1*(r2*r2*r2)*cos(q1-q2)+FxDes*I3*dq2*k1_11*m*r1*(r2*r2*r2)*cos(q1-q2)+FxDes*I3*dq1*k1_11*m*r1*(r2*r2*r2)*cos(q1+q2*3.0)+FxDes*I3*dq2*k1_11*m*r1*(r2*r2*r2)*cos(q1+q2*3.0)-FxDes*I3*dq1*dq2*m*r1*(r2*r2*r2)*sin(q1-q2)*4.0+FyDes*I3*dq1*k1_11*m*r1*(r2*r2*r2)*sin(q1-q2)+FyDes*I3*dq2*k1_11*m*r1*(r2*r2*r2)*sin(q1-q2)+FyDes*I3*dq1*k1_11*m*r1*(r2*r2*r2)*sin(q1+q2*3.0)+FyDes*I3*dq2*k1_11*m*r1*(r2*r2*r2)*sin(q1+q2*3.0)-I3*dq1*k1_11*ks*m*r1*(r2*r2)*cos(q2*2.0)*2.0-I3*dq2*k1_11*ks*m*r1*(r2*r2)*cos(q2*2.0)*2.0+I3*dq3*k1_11*ks*m*r1*(r2*r2)*cos(q2*2.0)*2.0-I3*dq1*dq2*ks*m*r1*(r2*r2)*sin(q2*2.0)*4.0-I3*k2_11*ks*m*q1*r1*(r2*r2)*cos(q2*2.0)*2.0-I3*k2_11*ks*m*q2*r1*(r2*r2)*cos(q2*2.0)*2.0+I3*k2_11*ks*m*q3*r1*(r2*r2)*cos(q2*2.0)*2.0)*(-1.0/2.0))/(ks*m*r1*(cos(q2*2.0)-1.0));
    double tauB = (1.0/(r1*r1)*(I6*(ks*ks)*q1*r2*2.0-I6*(ks*ks)*q6*r2*2.0-I6*(ks*ks)*q1*r1*cos(q2)*2.0-I6*(ks*ks)*q2*r1*cos(q2)*2.0+I6*(ks*ks)*q3*r1*cos(q2)*2.0+(ks*ks)*m*q1*(r1*r1)*r2-(ks*ks)*m*q6*(r1*r1)*r2+I6*cS*dq1*ks*r2*2.0-I6*cS*dq6*ks*r2*2.0-I6*cS*dq1*ks*r1*cos(q2)*2.0-I6*cS*dq2*ks*r1*cos(q2)*2.0+I6*cS*dq3*ks*r1*cos(q2)*2.0-c6*dq6*ks*m*(r1*r1)*r2+cS*dq1*ks*m*(r1*r1)*r2-cS*dq6*ks*m*(r1*r1)*r2-(ks*ks)*m*q1*(r1*r1)*r2*cos(q2*2.0)+(ks*ks)*m*q6*(r1*r1)*r2*cos(q2*2.0)+FxDes*I6*cS*dq1*(r1*r1)*cos(q1)*cos(q2)*2.0+FxDes*I6*cS*dq2*(r1*r1)*cos(q1)*cos(q2)*2.0-FxDes*I6*cS*dq3*(r1*r1)*cos(q1)*cos(q2)*2.0+FyDes*I6*(dq1*dq1)*m*(r1*r1*r1)*r2*cos(q1)-FxDes*I6*g*m*(r1*r1)*r2*pow(cos(q1),2.0)+FyDes*I6*cS*dq1*(r1*r1)*cos(q2)*sin(q1)*2.0+FyDes*I6*cS*dq2*(r1*r1)*cos(q2)*sin(q1)*2.0-FyDes*I6*cS*dq3*(r1*r1)*cos(q2)*sin(q1)*2.0+FxDes*I6*ks*q1*(r1*r1)*cos(q1)*cos(q2)*2.0+FxDes*I6*ks*q2*(r1*r1)*cos(q1)*cos(q2)*2.0-FxDes*I6*ks*q3*(r1*r1)*cos(q1)*cos(q2)*2.0-FxDes*I6*(dq1*dq1)*m*(r1*r1*r1)*r2*sin(q1)-FyDes*I6*g*m*(r1*r1)*r2*sin(q1*2.0)*(1.0/2.0)+FyDes*I6*ks*q1*(r1*r1)*cos(q2)*sin(q1)*2.0+FyDes*I6*ks*q2*(r1*r1)*cos(q2)*sin(q1)*2.0-FyDes*I6*ks*q3*(r1*r1)*cos(q2)*sin(q1)*2.0-I6*(dq1*dq1)*ks*m*r1*(r2*r2)*sin(q2)*2.0-I6*(dq2*dq2)*ks*m*r1*(r2*r2)*sin(q2)*2.0-FxDes*I6*cS*dq1*r1*r2*cos(q1)*2.0+FxDes*I6*cS*dq6*r1*r2*cos(q1)*2.0+c6*dq6*ks*m*(r1*r1)*r2*cos(q2*2.0)-cS*dq1*ks*m*(r1*r1)*r2*cos(q2*2.0)+cS*dq6*ks*m*(r1*r1)*r2*cos(q2*2.0)-FyDes*I6*cS*dq1*r1*r2*sin(q1)*2.0+FyDes*I6*cS*dq6*r1*r2*sin(q1)*2.0-FxDes*I6*ks*q1*r1*r2*cos(q1)*2.0+FxDes*I6*ks*q6*r1*r2*cos(q1)*2.0-I6*dq1*k1_22*ks*m*(r1*r1)*r2+I6*dq6*k1_22*ks*m*(r1*r1)*r2-FyDes*I6*ks*q1*r1*r2*sin(q1)*2.0+FyDes*I6*ks*q6*r1*r2*sin(q1)*2.0+I6*g*ks*m*r1*r2*cos(q1)-I6*k2_22*ks*m*q1*(r1*r1)*r2+I6*k2_22*ks*m*q6*(r1*r1)*r2-I6*(dq1*dq1)*ks*m*(r1*r1)*r2*sin(q2*2.0)-I6*g*ks*m*r1*r2*cos(q1+q2*2.0)-FyDes*I6*k2_22*m*(r1*r1*r1)*r2*cos(q1)+FxDes*I6*k2_22*m*(r1*r1*r1)*r2*sin(q1)+FyDes*I6*k2_22*m*(r1*r1*r1)*r2*cos(q2*2.0)*cos(q1)-FxDes*I6*k2_22*m*(r1*r1*r1)*r2*cos(q2*2.0)*sin(q1)+FxDes*I6*dq1*k1_22*m*(r1*r1*r1)*r2*cos(q1)+FyDes*I6*dq1*k1_22*m*(r1*r1*r1)*r2*sin(q1)-FyDes*I6*(dq1*dq1)*m*(r1*r1*r1)*r2*cos(q2*2.0)*cos(q1)-I6*dq1*dq2*ks*m*r1*(r2*r2)*sin(q2)*4.0+FxDes*I6*(dq1*dq1)*m*(r1*r1*r1)*r2*cos(q2*2.0)*sin(q1)+FxDes*I6*(dq1*dq1)*m*(r1*r1*r1)*r2*sin(q2*2.0)*cos(q1)+FxDes*I6*(dq1*dq1)*m*(r1*r1)*(r2*r2)*cos(q1)*sin(q2)*2.0+FxDes*I6*(dq2*dq2)*m*(r1*r1)*(r2*r2)*cos(q1)*sin(q2)*2.0+FyDes*I6*(dq1*dq1)*m*(r1*r1*r1)*r2*sin(q2*2.0)*sin(q1)+FyDes*I6*(dq1*dq1)*m*(r1*r1)*(r2*r2)*sin(q1)*sin(q2)*2.0+FyDes*I6*(dq2*dq2)*m*(r1*r1)*(r2*r2)*sin(q1)*sin(q2)*2.0+I6*dq1*k1_22*ks*m*(r1*r1)*r2*cos(q2*2.0)-I6*dq6*k1_22*ks*m*(r1*r1)*r2*cos(q2*2.0)+I6*k2_22*ks*m*q1*(r1*r1)*r2*cos(q2*2.0)-I6*k2_22*ks*m*q6*(r1*r1)*r2*cos(q2*2.0)+FxDes*I6*g*m*(r1*r1)*r2*cos(q1+q2*2.0)*cos(q1)+FyDes*I6*g*m*(r1*r1)*r2*cos(q1+q2*2.0)*sin(q1)-FxDes*I6*dq1*k1_22*m*(r1*r1*r1)*r2*cos(q2*2.0)*cos(q1)+FxDes*I6*dq1*dq2*m*(r1*r1)*(r2*r2)*cos(q1)*sin(q2)*4.0-FyDes*I6*dq1*k1_22*m*(r1*r1*r1)*r2*cos(q2*2.0)*sin(q1)+FyDes*I6*dq1*dq2*m*(r1*r1)*(r2*r2)*sin(q1)*sin(q2)*4.0))/(ks*m*r2*(cos(q2*2.0)-1.0));

    //// Desired motor angles
    //double q3des = 3.0*M_PI/4.0;
    //double q6des = M_PI/4.0;

    //// Compute the desired torque for a fixed motor position
    //double tauA = c3*dq3 - cS*dq1 - cS*dq2 + cS*dq3 - ks*q1 - ks*q2 + ks*q3 - I3*dq3*k1_11 - I3*k2_11*q3 + I3*k2_11*q3des;
    //double tauB = c6*dq6 - cS*dq1 + cS*dq6          - ks*q1 + ks*q6         - I6*dq6*k1_22 - I6*k2_22*q6 + I6*k2_22*q6des;

    // Convert desired torque to current using the gear ratio (KG) and torque
    // constant (KT)
    double curA = tauA/KG/KT;
    double curB = tauB/KG/KT;

    // Z-direction is opposite in simulation
    curA = -curA;
    curB = -curB;

    //printf("FxDes: %f\n", FxDes);
    //printf("FyDes: %f\n", FyDes);
    //printf("q1: %f\n", q1);
    //printf("q2: %f\n", q2);
    //printf("q3: %f\n", q3);
    //printf("q6: %f\n", q6);
    //printf("tauA: %f\n", tauA);
    //printf("tauB: %f\n", tauB);
    //printf("\n");

    // Log the desired forces
    logOut.fxDes = FxDes;
    logOut.fzDes = FyDes;

    // Return the motor current
    return std::make_tuple(curA, curB);
}

// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCForceControlDemo)

}
}

// vim: noexpandtab
