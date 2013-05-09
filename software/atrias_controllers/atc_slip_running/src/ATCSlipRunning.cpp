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
	ascInterpolation(this, "ascInterpolation"),
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
					
					// Compute current leg angle and length
					std::tie(qRl, rRl) = ascCommonToolkit.motorPos2LegPos(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle);					
					// Compute current leg cartesian components lengths
					std::tie(xRl, zRl) = ascCommonToolkit.polPos2CartPos(qRl, rRl);					
					// Switch to next state if touchdown
					if (rs.position.zPosition < zRl) {
						runningState = 1;
					}					
					break;
				
				// Right leg stance
				case 1:
					rightLegStance();
					
					// Compute current leg angle and length
					std::tie(qRl, rRl) = ascCommonToolkit.motorPos2LegPos(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle);					
					// Compute current leg cartesian components lengths
					std::tie(xRl, zRl) = ascCommonToolkit.polPos2CartPos(qRl, rRl);					
					// Switch to next state if takeoff
					if (rs.position.zPosition > zRl) {
						runningState = 2;
						// Compute target state and time until apex						
						t1 = 0.0;
						t2 = rs.position.zVelocity/G;
						std::tie(qLl1, rLl1) = ascCommonToolkit.motorPos2LegPos(rs.lLeg.halfA.legAngle, rs.lLeg.halfB.legAngle);
						std::tie(qRl1, rRl1) = ascCommonToolkit.motorPos2LegPos(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle);
						std::tie(dqLl1, drLl1) = ascCommonToolkit.motorVel2LegVel(rs.lLeg.halfA.legAngle, rs.lLeg.halfB.legAngle, rs.lLeg.halfA.legVelocity, rs.lLeg.halfB.legVelocity);
						std::tie(dqRl1, drRl1) = ascCommonToolkit.motorVel2LegVel(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle, rs.rLeg.halfA.legVelocity, rs.rLeg.halfB.legVelocity);											
						qLl2 = equilibriumGaitSolver(rs.position,xVelocity, rs.position,zVelocity, rLl1);
						dqLl2 = 0.0;
						
						rLl2 = 0.75;
						drLl2 = 0;
						
						rRl2 = 0;
						drRl2 = 0;	

					}
					break;
					
				// Left leg flight - rising
				case 2:
					leftLegFlightRising();
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
					
					// Switch to next state if apex has been reached
					if (rs.position.zVelocity < 0.0) { // TODO: may need to debounce
						runningState = 0;
						// Compute target state and time until touch down
						t1 = 0.0;
						t2 = rs.position.zVelocity/G;// TODO: fix using height
						std::tie(qLl1, rLl1) = ascCommonToolkit.motorPos2LegPos(rs.lLeg.halfA.legAngle, rs.lLeg.halfB.legAngle);
						std::tie(qRl1, rRl1) = ascCommonToolkit.motorPos2LegPos(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle);
						std::tie(dqLl1, drLl1) = ascCommonToolkit.motorVel2LegVel(rs.lLeg.halfA.legAngle, rs.lLeg.halfB.legAngle, rs.lLeg.halfA.legVelocity, rs.lLeg.halfB.legVelocity);
						std::tie(dqRl1, drRl1) = ascCommonToolkit.motorVel2LegVel(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle, rs.rLeg.halfA.legVelocity, rs.rLeg.halfB.legVelocity);
						
						rLl2 = 0;
						drLl2 = 0;
						
						rRl2 = 0.85;
						drRl2 = 0;	
					}
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

	// Get GUI values
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

	// Advance time counter
	dt = ((double) CONTROLLER_LOOP_PERIOD_NS) / ((double) SECOND_IN_NANOSECONDS); 
	t = t + dt;
	
	// Redefine slip initial conditions incase we go into stance next time step
	std::tie(slipState.q, slipState.r) = ascCommonToolkit.motorPos2LegPos(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle);
	std::tie(slipState.dq, slipState.dr) = ascCommonToolkit.cartVel2PolVel(slipState.q, slipState.r, rs.position.xVelocity, rs.position.zVelocity);
		
		
	// Right leg control
		// Cubic interpolation to extended leg
		std::tie(rRl, drRl) = ascInterpolation.cubic(t1, t2, rRl1, rRl2, drRl1, drRl2, t); // TODO: may want to set target velocity to something other than zero. Could check equilibrium gait curevfit derivatives to find correct velocity to match ground speed.
	
		// Let the leg overextend past target location but limit at extremes
		rRl = clamp(rRl, 0.5, 0.95);
	
		// Compute leg angle to give equalibrium gait
		qRl = equilibriumGaitSolver(rs.position.xVelocity, rs.position.zVelocity, rRl);
		dqRl = 0.0; // TODO: need to add lookup value in gait solver for velocity.

		// Set leg motor angles
		std::tie(qRmA, qRmB) = ascCommonToolkit.legPos2MotorPos(qRl, rRl);
		std::tie(dqRmA, dqRmB) = ascCommonToolkit.legVel2MotorVel(rRl, dqRl, drRl);
		
		// Compute and set motor currents
		co.rLeg.motorCurrentA = ascPDRmA(qRmA, rs.rLeg.halfA.motorAngle, dqRmA, rs.rLeg.halfA.motorVelocity);
		co.rLeg.motorCurrentB = ascPDRmB(qRmB, rs.rLeg.halfB.motorAngle, dqRmB, rs.rLeg.halfB.motorVelocity);
	
	
	// Left leg control
		// Cubic interpolation to shorten leg
		std::tie(rLl, drLl) = ascInterpolation.cubic(t1, t2, rLl1, rLl2, drLl1, drLl2, t);
	
		// Let the leg overcontract past target location but limit at extremes
		rLl = clamp(rLl, 0.5, 0.95);
	
		// Mirror other leg angle and velocity
		qLl = PI - qRl;
		dqLl = -dqRl;
	
		// Set leg motor angles
		std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(qLl, rLl);
		std::tie(dqLmA, dqLmB) = ascCommonToolkit.legVel2MotorVel(rLl, dqLl, drLl);

		// Compute and set motor currents
		co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, dqLmA, rs.lLeg.halfA.motorVelocity);
		co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, dqLmB, rs.lLeg.halfB.motorVelocity);

}


void ATCSlipRunning::rightLegStance() {
	
	// Compute current virtual leg length spring stiffness
	std::tie(ascSlipModel.k, dk) = ascCommonToolkit.legStiffness(slipState.r, ascSlipModel.r0);

	// Advance SLIP model integrator
	slipState = ascSlipModel.advanceRK5(slipState);
	
	// Compute simulated SLIP model forces
	legForce = ascSlipModel.force(slipState);


	// Right leg control
		// If SLIP model says we should be in flight then implement a position controller to prevent collapse
		if (slipState.isFlight) {
			// Use last know leg position from stance
			co.rLeg.motorCurrentA = ascPDRmA(qRmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
			co.rLeg.motorCurrentB = ascPDRmB(qRmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);

		// If SLIP model says we should be in stance then implement a force controller to track SLIP model forces
		} else {
			// Store last known leg position so we can hold this position if we don't make it back into flight
			qRmA = rs.rLeg.halfA.legAngle;
			qRmB = rs.rLeg.halfB.legAngle;

			// Compute and set motor currents
			std::tie(co.rLeg.motorCurrentA, co.rLeg.motorCurrentB) = ascLegForceRl.control(legForce, rs.rLeg, rs.position);

		}


	// Left leg control
		// Leave leg length constant TODO: may want to add cubic interpolation to shorten further
		rLl = rLl;
		drLl = 0.0;
	
		// Mirror other leg angle and velocity
		std::tie(qRl, rRl) = ascCommonToolkit.motorPos2LegPos(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle);
		std::tie(dqRl, drRl) = ascCommonToolkit.motorVel2LegVel(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle, rs.rLeg.halfA.legVelocity, rs.rLeg.halfB.legVelocity);
		qLl = PI - qRl;
		dqLl = - dqRl;
	
		// Set leg motor angles and velocities
		std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(qLl, rLl);
		std::tie(dqLmA, dqLmB) = ascCommonToolkit.legVel2MotorVel(rLl, dqLl, drLl);

		// Compute and set motor currents
		co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, dqLmA, rs.lLeg.halfA.motorVelocity);
		co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, dqLmB, rs.lLeg.halfB.motorVelocity);

}


void ATCSlipRunning::leftLegFlightRising() {
	
	// Advance time counter
	dt = ((double) CONTROLLER_LOOP_PERIOD_NS) / ((double) SECOND_IN_NANOSECONDS); 
	t = t + dt;

	// Left leg control
		// Cubic interpolation to extended and protract leg
		std::tie(rLl, drLl) = ascInterpolation.cubic(t1, t2, rLl1, rLl2, drLl1, drLl2, t);
		std::tie(qLl, dqLl) = ascInterpolation.cubic(t1, t2, qLl1, qLl2, dqLl1, dqLl2, t);
	
		// Let the leg overextend/protract past target location but limit at extremes
		rLl = clamp(rLl, 0.5, 0.95);
		qLl = clamp(qLl, 0.0, PI); // TODO: determine resonable values or if it is even needed for angles

		// Set leg motor angles and velocities
		std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(qLl, rLl);
		std::tie(dqLmA, dqLmB) = ascCommonToolkit.legVel2MotorVel(rLl, dqLl, drLl);
	
		// Compute and set motor currents
		co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, dqLmA, rs.lLeg.halfA.motorVelocity);
		co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, dqLmB, rs.lLeg.halfB.motorVelocity);
		
	
	// Right leg control
		// Cubic interpolation to shorten and retract leg
		std::tie(rRl, drRl) = ascInterpolation.cubic(t1, t2, rRl1, rRl2, drRl1, drRl2, t); // TODO: Need to further define this
	
		// Let the leg overretract past target location but limit at extremes
		rRl = clamp(rRl, 0.5, 0.95);
	
		// Mirror other leg angle and velocity
		qRl = PI - qLl;	// TODO: maybe, investigate if this is effective or if it should be an interp as well
		dqRl = -dqLl;
	
		// Set leg motor angles and velocities
		std::tie(qRmA, qRmB) = ascCommonToolkit.legPos2MotorPos(qRl, rRl);
		std::tie(dqRmA, dqRmB) = ascCommonToolkit.legVel2MotorVel(rRl, dqRl, drRl);

		// Compute and set motor currents
		co.rLeg.motorCurrentA = ascPDRmA(qRmA, rs.rLeg.halfA.motorAngle, dqRmA, rs.rLeg.halfA.motorVelocity);
		co.rLeg.motorCurrentB = ascPDRmB(qRmB, rs.rLeg.halfB.motorAngle, dqRmB, rs.rLeg.halfB.motorVelocity);

}


void ATCSlipRunning::leftLegFlightFalling() {

	// Opposite as rightLegFlightFalling

}


void ATCSlipRunning::leftLegStance() {

	// Opposite as rightLegStance
	
}


void ATCSlipRunning::rightLegFlightRising() {

	// Opposite as rightLegFlightRising

}


double ATCSlipRunnig::equilibriumGaitSolver(double dx, double dz, double r0) {

	b[0] = -7.213440413060714;
	b[1] = 3.468798283842735E1;
	b[2] = -3.266444726973395E1;
	b[3] = -2.702469175556414E1;
	b[4] = 5.780176026187765E1;
	b[5] = -2.403672244571941E1;
	b[6] = -4.515544770572114;
	b[7] = 2.791907849832265E1;
	b[8] = -5.826582192003637E1;
	b[9] = 5.116132452827277E1;
	b[10] = -1.628190619067855E1;
	b[11] = 1.004520907772565;
	b[12] = -2.734630129003093;
	b[13] = 2.439719593593622;
	b[14] = -6.768833584634145E-1;
	b[15] = 7.319350216643417E-2;
	b[16] = -1.692287640834765E-1;
	b[17] = 1.073350427329765E-1;
	b[18] = -2.378847271765413E-3;
	b[19] = 3.589351794506902E-3;
	b[20] = 1.848047074272373E-5;
	b[21] = 7.724980660436322;
	b[22] = -3.439729952637526E1;
	b[23] = 5.913089242725211E1;
	b[24] = -4.47845055059484E1;
	b[25] = 1.239865396732889E1;
	b[26] = 8.280999797829944E-2;
	b[27] = -1.063607081263741;
	b[28] = 2.06342756269128;
	b[29] = -1.060098804785176;
	b[30] = -1.464456757236583E-1;
	b[31] = 2.261792541244291E-1;
	b[32] = -6.401309879432286E-2;
	b[33] = -1.308951787975762E-2;
	b[34] = 1.463995370269994E-2;
	b[35] = -2.66004349934331E-5;
	b[36] = -2.204247217557561E-1;
	b[37] = 5.585818255173174E-1;
	b[38] = -6.761674427762996E-1;
	b[39] = 3.172038249573401E-1;
	b[40] = -1.384541440998458E-2;
	b[41] = -2.396643759197579E-2;
	b[42] = 1.575640956987011E-2;
	b[43] = 1.130596092328483E-3;
	b[44] = -5.056850385043376E-3;
	b[45] = -6.911746916972324E-5;
	b[46] = 7.614291839129032E-3;
	b[47] = -2.274979808651478E-3;
	b[48] = -3.210690228678975E-3;
	b[49] = 3.100649534001704E-3;
	b[50] = -8.041932324654042E-4;
	b[51] = 2.487263753304149E-4;
	b[52] = -1.922449704847737E-4;
	b[53] = 1.864365446186798E-4;
	b[54] = -7.304085547540021E-5;
	b[55] = -5.26651489998567E-6;
  
	// Curve fit of equilibrium gait solution for ATRIAS
	q = b[1] + b[22]*dx + b[7]*dz + b[2]*r0 + b[37]*(dx*dx) + b[47]*(dx*dx*dx) + b[53]*(dx*dx*dx*dx) + b[56]*(dx*dx*dx*dx*dx) + b[12]*(dz*dz) + b[16]*(dz*dz*dz) + b[19]*(dz*dz*dz*dz) + b[21]*(dz*dz*dz*dz*dz) + b[3]*(r0*r0) + b[4]*(r0*r0*r0) + b[5]*(r0*r0*r0*r0) + b[6]*(r0*r0*r0*r0*r0) + b[44]*(dx*dx)*(dz*dz) + b[46]*(dx*dx)*(dz*dz*dz) + b[52]*(dx*dx*dx)*(dz*dz) + b[39]*(dx*dx)*(r0*r0) + b[40]*(dx*dx)*(r0*r0*r0) + b[49]*(dx*dx*dx)*(r0*r0) + b[14]*(dz*dz)*(r0*r0) + b[15]*(dz*dz)*(r0*r0*r0) + b[18]*(dz*dz*dz)*(r0*r0) + b[27]*dx*dz + b[23]*dx*r0 + b[8]*dz*r0 + b[31]*dx*(dz*dz) + b[34]*dx*(dz*dz*dz) + b[36]*dx*(dz*dz*dz*dz) + b[41]*(dx*dx)*dz + b[50]*(dx*dx*dx)*dz + b[55]*(dx*dx*dx*dx)*dz + b[24]*dx*(r0*r0) + b[25]*dx*(r0*r0*r0) + b[26]*dx*(r0*r0*r0*r0) + b[38]*(dx*dx)*r0 + b[48]*(dx*dx*dx)*r0 + b[54]*(dx*dx*dx*dx)*r0 + b[9]*dz*(r0*r0) + b[10]*dz*(r0*r0*r0) + b[11]*dz*(r0*r0*r0*r0) + b[13]*(dz*dz)*r0 + b[17]*(dz*dz*dz)*r0 + b[20]*(dz*dz*dz*dz)*r0 + b[29]*dx*dz*(r0*r0) + b[30]*dx*dz*(r0*r0*r0) + b[32]*dx*(dz*dz)*r0 + b[35]*dx*(dz*dz*dz)*r0 + b[42]*(dx*dx)*dz*r0 + b[51]*(dx*dx*dx)*dz*r0 + b[33]*dx*(dz*dz)*(r0*r0) + b[43]*(dx*dx)*dz*(r0*r0) + b[45]*(dx*dx)*(dz*dz)*r0 + b[28]*dx*dz*r0;

	// TODO: Probably don't need immediately but velocity will increase tracking performance
	dq = 0;

	// Return our output command
	// return std::make_tuple(q, dq);
	return q;	

}


// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCSlipRunning)

}
}
