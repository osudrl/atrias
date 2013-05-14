/**
 * @file ATCHeuristicSlipWalking.cpp
 * @author Mikhail S. Jones
 * @brief This controller is based on simulated SLIP walking gaits for ATRIAS and has been tuned manually through trial and error.
 */

#include "atc_heuristic_slip_walking/ATCHeuristicSlipWalking.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

ATCHeuristicSlipWalking::ATCHeuristicSlipWalking(string name) :
	ATC(name),
	ascCommonToolkit(this, "ascCommonToolkit"),
	ascInterpolation(this, "ascInterpolation"),
	ascSlipModel(this, "ascSlipModel"),
	ascLegForceL(this, "ascLegForceL"),
	ascLegForceR(this, "ascLegForceR"),
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
	legRateLimit = 0.5;
	
	// Set hip controller toe positions
	toePosition.left = 2.15;
	toePosition.right = 2.45;

	// Initialize walking state
	//controllerState = 0;
	walkingState = 0;
}


void ATCHeuristicSlipWalking::controller() {
	// Startup is handled by the ATC class
	setStartupEnabled(true);

	// Update current robot state
	updateState();

	// Run hip controller
	hipController();

	// Main controller state machine
	switch (controllerState) {
		case 0: // Stand upright in place
			standingController(&rs.rLeg, &co.rLeg, &ascRateLimitRmA, &ascRateLimitRmB, &ascPDRmA, &ascPDRmB);
			standingController(&rs.lLeg, &co.lLeg, &ascRateLimitLmA, &ascRateLimitLmB, &ascPDLmA, &ascPDLmB);
			// Reset walking state parameters
			walkingState = 0;
			qeSl = PI/2.0;
			qeFl = PI/2.0;
			reFl = guiIn.standing_leg;
			break;

		case 1: // SLIP heuristic walking
			// SLIP heuristic walking controller state machine
			switch (walkingState) {			
				case 0: // Right leg single support	(right = stance, left = flight)
					stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR);
					legSwingController(&rs.rLeg, &rs.lLeg, &co.lLeg, &ascPDLmA, &ascPDLmB);
					singleSupportEvents(&rs.rLeg, &rs.lLeg);
					break;
			
				case 1: // Double support (right = flight, left = stance)
					stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR);
					stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL);
					doubleSupportEvents(&rs.lLeg, &rs.rLeg);
					break;
				
				case 2: // Left leg single support (right = flight, left = stance)
					stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL);
					legSwingController(&rs.lLeg, &rs.rLeg, &co.rLeg, &ascPDRmA, &ascPDRmB);
					singleSupportEvents(&rs.lLeg, &rs.rLeg);
					break;
				
				case 3: // Double support (right = stance, left = flight)
					stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL);
					stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR);
					doubleSupportEvents(&rs.rLeg, &rs.lLeg);
					break;
			}
			break;
		
		case 2: // Shutdown
			shutdownController(); // Call shutdown controller
			break;
	}

}


void ATCHeuristicSlipWalking::updateState() {
	// Reset rate limiters when not enabled (prevents jumps)
	if (!isEnabled()) {
		ascRateLimitLmA.reset(rs.lLeg.halfA.motorAngle);
		ascRateLimitLmB.reset(rs.lLeg.halfB.motorAngle);
		ascRateLimitRmA.reset(rs.rLeg.halfA.motorAngle);
		ascRateLimitRmB.reset(rs.rLeg.halfB.motorAngle);
	}

	// Get some gui inputs
	controllerState = guiIn.main_controller;
	r0 = guiIn.slip_leg;
	
	// Switch betwwen manual event trigger from GUI (pull walking state), or automatic (push walking state)
	if (guiIn.state_trigger == 0) {
		walkingState = guiIn.walking_state;
	} else if (guiIn.state_trigger == 1) {
		guiOut.walking_state = walkingState;
	}	

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

	// Compute actual leg force from spring deflection and robot state
	ascLegForceL.compute(rs.lLeg, rs.position);
	ascLegForceR.compute(rs.rLeg, rs.position);

}


void ATCHeuristicSlipWalking::hipController() {
	// Compute inverse kinematics to keep lateral knee torque to a minimum
	std::tie(qLh, qRh) = ascHipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);

	// Compute and set motor currents from position based PD controllers
	co.lLeg.motorCurrentHip = ascPDLh(qLh, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentHip = ascPDRh(qRh, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);
}


void ATCHeuristicSlipWalking::standingController(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::controller_output_leg *coSl, ASCRateLimit *ascRateLimitmA, ASCRateLimit *ascRateLimitmB, ASCPD *ascPDmA, ASCPD *ascPDmB) {
	// Set leg angles for a neutral upright standing pose
	qSl = PI/2.0;
	rSl = guiIn.standing_leg;

	// Compute required motor angles
	std::tie(qmSA, qmSB) = ascCommonToolkit.legPos2MotorPos(qSl, rSl);

	// Rate limit motor velocities to smooth step inputs
	qmSA = ascRateLimitmA->operator()(qmSA, legRateLimit);
	qmSB = ascRateLimitmB->operator()(qmSB, legRateLimit);

	// Compute and set motor currents from position based PD controllers
	coSl->motorCurrentA = ascPDmA->operator()(qmSA, rsSl->halfA.motorAngle, 0.0, rsSl->halfA.motorVelocity);
	coSl->motorCurrentB = ascPDmB->operator()(qmSB, rsSl->halfB.motorAngle, 0.0, rsSl->halfB.motorVelocity);
}


void ATCHeuristicSlipWalking::shutdownController() {
	// Compute and set motor currents (applies virtual dampers to all actuators)
	co.lLeg.motorCurrentA = ascPDLmA(0.0, 0.0, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDLmB(0.0, 0.0, 0.0, rs.lLeg.halfB.motorVelocity);
	co.lLeg.motorCurrentHip = ascPDLh(0.0, 0.0, 0.0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentA = ascPDRmA(0.0, 0.0, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDRmB(0.0, 0.0, 0.0, rs.rLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentHip = ascPDRh(0.0, 0.0, 0.0, rs.rLeg.hip.legBodyVelocity);
}


void ATCHeuristicSlipWalking::stanceController(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::controller_output_leg *coSl, ASCLegForce *ascLegForce) {
	// Compute current ATRIAS non-linear spring constant for given leg configuration
	std::tie(k, dk) = ascCommonToolkit.legStiffness(rSl, drSl, r0);

	// Compute current leg angle and length
	std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
	std::tie(dqSl, drSl) = ascCommonToolkit.motorVel2LegVel(rsSl->halfA.legAngle, rsSl->halfB.legAngle, rsSl->halfA.legVelocity, rsSl->halfB.legVelocity);

	// Define component forces and their derivatives
	legForce.fx = -k*(rSl - r0)*cos(qSl);
	legForce.dfx = dk*cos(qSl)*(r0 - rSl) - drSl*cos(qSl)*k + dqSl*sin(qSl)*k*(rSl - r0);
	legForce.fz = k*(rSl - r0)*sin(qSl);
	legForce.dfz = drSl*sin(qSl)*k - dk*sin(qSl)*(r0 - rSl) + dqSl*cos(qSl)*k*(rSl - r0);
	
	// Use force tracking controller to compute required motor currents
	std::tie(coSl->motorCurrentA, coSl->motorCurrentB) = ascLegForce->control(legForce, *rsSl, rs.position);
}

void ATCHeuristicSlipWalking::legSwingController(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::robot_state_leg *rsFl, atrias_msgs::controller_output_leg *coFl, ASCPD *ascPDmA, ASCPD *ascPDmB) {
	// A gait is defined by
		// The TO angle of the flight leg
		// The TO length of the flight leg (not as important, could use to inject energy)
		// The TD angle of the flight leg (after constants work could fit curve fro various gaits vs dx)
		// The TD length of the flight leg (should be constant)
		// The angle of the stance leg when TO of flight leg occurs
		// The simulated angle of the stance leg during flight TD
		// Amount of flight leg retraction desired
		// Velocities of cubic splines can be adjusted as needed.

	// Some good gaits found through simulation on MATLAB
		// r0 = 0.80, qtFl = PI - (PI/2.0 + 0.230), qtSl = PI - (PI/2.0 - 0.10), legRetraction = 0.10
		// r0 = 0.85, qtFl = PI - (PI/2.0 + 0.193), qtSl = PI - (PI/2.0 - 0.07), legRetraction = 0.07
		// r0 = 0.90, qtFl = PI - (PI/2.0 + 0.157), qtSl = PI - (PI/2.0 - 0.05), legRetraction = 0.05
		
	// TODO: Should make angles relative to world not to body.
	// TODO: If qeSl > qtSl then we have a issue, need to error catch but not sure what to do if it fails (maybe just take current value and add on some resonable amount)
	
	// Compute the current leg angles and lengths
	std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
	std::tie(qFl, rFl) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.legAngle, rsFl->halfB.legAngle);

	// Use a cubic spline interpolation to slave the flight leg angle and length to the stance leg angle
	qtSl = PI/2.0 + 0.1; // Predicted stance leg angle at flight leg TD
	qtFl = PI/2.0 - 0.23; // Target flight leg angle at TD
	std::tie(ql, dql) = ascInterpolation.cubic(qeSl, qtSl, qeFl, qtFl, 0, 0, qSl);

	// Use two cubic splines slaved to stance leg angle to retract and then extend the flight leg
	if (qSl <= (qeSl + qtSl)/2.0) {
		// Leg retraction during first half
		std::tie(rl, drl) = ascInterpolation.cubic(qeSl, (qeSl + qtSl)/2.0, reFl, r0 - 0.1, 0, 0, qSl); // 
	} else {
		// Leg extension during the second half
		std::tie(rl, drl) = ascInterpolation.cubic((qeSl + qtSl)/2.0, qtSl, r0 - 0.1, r0, 0, 0, qSl);
	}

	// Convert desired leg angles and lengths into motor positions and velocities
	std::tie(qmFA, qmFB) = ascCommonToolkit.legPos2MotorPos(ql, rl);
	std::tie(dqmFA, dqmFB) = ascCommonToolkit.legVel2MotorVel(rl, dql, drl);	

	// Compute and set motor currents from position based PD controllers
	coFl->motorCurrentA = ascPDmA->operator()(qmFA, rsFl->halfA.motorAngle, dqmFA, rsFl->halfA.motorVelocity);
	coFl->motorCurrentB = ascPDmB->operator()(qmFB, rsFl->halfB.motorAngle, dqmFB, rsFl->halfB.motorVelocity);

}


// TODO: switch over to the toe ground contact sensors once they are ready
void ATCHeuristicSlipWalking::singleSupportEvents(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::robot_state_leg *rsFl) {
	// Compute the current flight leg angle and length
	std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
	std::tie(qFl, rFl) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.legAngle, rsFl->halfB.legAngle);

	// Compute conditionals for event triggers
	isFlightLegTD = (rFl*sin(qFl) >= rs.position.zPosition);
	isForwardStep = (rSl*cos(qSl) <= rFl*cos(qFl));
	isBackwardStep = (rSl*cos(qSl) > rFl*cos(qFl));
	
	// Flight leg touch down event (trigger next state)
	if (isFlightLegTD && isForwardStep) {
		// Advance the walking state machine 1 step and loop to beginning if needed
		walkingState = (walkingState + 1) % 4;
		printf("singleSupportEvent TD forward step triggered - walkingState = %i\n", walkingState); // TODO: remove after debugging
	} else if (isFlightLegTD && isBackwardStep) {
		// Regress the walking state machine 1 step and loop to end if needed (the + 4 prevents negatives)
		walkingState = (walkingState - 1 + 4) % 4;
		printf("singleSupportEvent TD backward step triggered - walkingState = %i\n", walkingState); // TODO: remove after debugging
	}

	// Stance leg take off event (ignore)
	// TODO: add trigger to catch, this would mean the robot is bouncing, nothing to do to really stop it though

}

// TODO: switch over to the toe ground contact sensors once they are ready
void ATCHeuristicSlipWalking::doubleSupportEvents(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::robot_state_leg *rsFl) {
	// Compute the current flight leg angle and length
	std::tie(qFl, rFl) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.legAngle, rsFl->halfB.legAngle);

	// Compute conditionals for event triggers
	isFlightLegTO = (rFl*sin(qFl) <= rs.position.zPosition);
	
	// Flight leg take off (trigger next state)
	if (isFlightLegTO) {
		// Advance the walking state machine 1 step and loop to beginning if needed
		walkingState = (walkingState + 1) % 4;
		printf("doubleSupportEvent TO triggered walkingState = %i\n", walkingState); // TODO: remove after debugging
		
		// Save the stance and flight leg exit conditions for use in leg swing trajectory generation
		std::tie(qeFl, reFl) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.legAngle, rsFl->halfB.legAngle);
		std::tie(dqeFl, dreFl) = ascCommonToolkit.motorVel2LegVel(rsFl->halfA.legAngle, rsFl->halfB.legAngle, rsFl->halfA.legVelocity, rsFl->halfB.legVelocity);
		std::tie(qeSl, reSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
		std::tie(dqeSl, dreSl) = ascCommonToolkit.motorVel2LegVel(rsSl->halfA.legAngle, rsSl->halfB.legAngle, rsSl->halfA.legVelocity, rsSl->halfB.legVelocity);
	}

	// Stance leg take off (ignore)
	// TODO: add trigger to catch, this means we started going backwards
}


ORO_CREATE_COMPONENT(ATCHeuristicSlipWalking)

}
}
