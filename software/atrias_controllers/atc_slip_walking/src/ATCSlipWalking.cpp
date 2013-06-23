/**
 * @file ATCSlipWalking.cpp
 * @brief A Spring Loaded Inverted Pendulum (SLIP) template model based
 * walking controller.
 * @author Mikhail Jones
 */

#include "atc_slip_walking/ATCSlipWalking.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

ATCSlipWalking::ATCSlipWalking(string name) :
    ATC(name),
    ascCommonToolkit(this, "ascCommonToolkit"),
    ascHipBoomKinematics(this, "ascHipBoomKinematics"),
    ascInterpolation(this, "ascInterpolation"),
    ascLegForceR(this, "ascLegForceR"),
    ascLegForceL(this, "ascLegForceL"),
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
    ascRateLimitRh(this, "ascRateLimitRh"),
    ascRateLimitLr0(this, "ascRateLimitLr0"),
    ascRateLimitRr0(this, "ascRateLimitRr0")
{
    // Startup is handled by the ATC class
    setStartupEnabled(true);

    // Set leg motor rate limit
    legRateLimit = 0.5;
    hipRateLimit = 0.5;
    springRateLimit = 0.25;

    // Initialize walking state
    walkingState = 0;
}

/**
 * @brief Top-level controller.
 * 
 * This is the main function for the top-level controller.
 * The ATC class automatically handles startup and shutdown,
 * if they are not disabled.
 */
void ATCSlipWalking::controller() {
    // Update GUI values, gains, and other options
    updateController();

    // Run the hip controller
    hipController();

    // Main controller state machine
    switch (controllerState) {

        case 0: // Stand upright in place
            // Call standing controller
            standingController();

            // Reset walking state parameters
            walkingState = 0;

            // Save the stance and flight leg exit conditions (right = stance, left = flight)
            updateExitConditions(&rs.rLeg, &rs.lLeg, &ascRateLimitRr0, &ascRateLimitLr0);
            break;

        case 1: // SLIP  walking
            // SLIP  walking controller state machine
            switch (walkingState) {

                case 0: // Right leg single support (right = stance, left = flight)
                    // Run state specific controller and event functions
                    legSwingController(&rs.rLeg, &rs.lLeg, &co.lLeg, &ascPDLmA, &ascPDLmB);
                    stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR, &ascRateLimitRr0);
                    singleSupportEvents(&rs.rLeg, &rs.lLeg, &ascLegForceR, &ascLegForceL, &ascRateLimitRr0, &ascRateLimitLr0);

			  // Inject energy
			  if (qSl <= qtSl) {
			      co.rLeg.motorCurrentA += -guiIn.pushoff_force;
			  	co.rLeg.motorCurrentB += -guiIn.pushoff_force;
			  } else {
				walkingState = (walkingState + 1) % 4;
			  }
                    break;

                case 1: // Double support (right = flight, left = stance)
                    // Run state specific controller and event functions
                    stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR, &ascRateLimitRr0);
                    stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL, &ascRateLimitLr0);
                    doubleSupportEvents(&rs.lLeg, &rs.rLeg, &ascLegForceL, &ascLegForceR, &ascRateLimitLr0, &ascRateLimitRr0);

			  // Inject energy
			  co.lLeg.motorCurrentA += -5.0;
                    co.lLeg.motorCurrentB += -5.0;
                    break;

                case 2: // Left leg single support (right = flight, left = stance)
                    // Run state specific controller and event functions
                    legSwingController(&rs.lLeg, &rs.rLeg, &co.rLeg, &ascPDRmA, &ascPDRmB);
                    stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL, &ascRateLimitLr0);
                    singleSupportEvents(&rs.lLeg, &rs.rLeg, &ascLegForceL, &ascLegForceR, &ascRateLimitLr0, &ascRateLimitRr0);

			  // Inject energy
			  if (qSl <= qtSl) {
			      co.lLeg.motorCurrentA += -guiIn.pushoff_force;
			  	co.lLeg.motorCurrentB += -guiIn.pushoff_force;
			  } else {
                        walkingState = (walkingState + 1) % 4;
			  }
                    break;

                case 3: // Double support (right = stance, left = flight)
                    // Run state specific controller and event functions
                    stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL, &ascRateLimitLr0);
                    stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR, &ascRateLimitRr0);
                    doubleSupportEvents(&rs.rLeg, &rs.lLeg, &ascLegForceR, &ascLegForceL, &ascRateLimitRr0, &ascRateLimitLr0);
			  
			  // Inject energy
			  co.rLeg.motorCurrentA += -5.0;
                    co.rLeg.motorCurrentB += -5.0;
                    break;
            }
            break;

        case 2: // Shutdown
            // Call shutdown controller
            shutdownController();

            // Reset walking state parameters
            walkingState = 0;

            // Save the stance and flight leg exit conditions (right = stance, left = flight)
            updateExitConditions(&rs.rLeg, &rs.lLeg, &ascRateLimitRr0, &ascRateLimitLr0);
            break;
    }
}

/**
 * @brief Update controller parameters.
 * 
 * This function handles all of the non-controller related
 *  updating and all communication to and from the GUI.
 */
void ATCSlipWalking::updateController() {
    // If we are disabled or the controller has been switched, reset rate limiters
    if (!(isEnabled()) || !(controllerState == guiIn.main_controller)) {
        ascRateLimitLmA.reset(rs.lLeg.halfA.motorAngle);
        ascRateLimitLmB.reset(rs.lLeg.halfB.motorAngle);
        ascRateLimitRmA.reset(rs.rLeg.halfA.motorAngle);
        ascRateLimitRmB.reset(rs.rLeg.halfB.motorAngle);
        ascRateLimitLh.reset(rs.lLeg.hip.legBodyAngle);
        ascRateLimitRh.reset(rs.rLeg.hip.legBodyAngle);
        ascRateLimitLr0.reset(r0);
        ascRateLimitRr0.reset(r0);
    }

    // Main controller options
    controllerState = guiIn.main_controller;

    // Gait options
    ascCommonToolkit.ks = guiIn.atrias_spring;
    r0 = guiIn.slip_leg;
    qtSl = guiIn.stance_leg_target;
    qtFl = guiIn.flight_leg_target;
    swingLegRetraction = guiIn.swing_leg_retraction;

    // Event and state trigger parameters
    forceThresholdTO = guiIn.force_threshold_td;
    forceThresholdTO = guiIn.force_threshold_to;
    positionThresholdTD = guiIn.position_threshold_td;

    // Leg gains
    ascPDLmA.P = ascPDLmB.P = ascPDRmA.P = ascPDRmB.P = guiIn.leg_pos_kp;
    ascPDLmA.D = ascPDLmB.D = ascPDRmA.D = ascPDRmB.D = guiIn.leg_pos_kd;
    ascLegForceL.kp = ascLegForceR.kp = guiIn.leg_for_kp;
    ascLegForceL.ki = ascLegForceR.ki = guiIn.leg_for_ki;
    ascLegForceL.kd = ascLegForceR.kd = guiIn.leg_for_kd;

    // Hip gains
    ascPDLh.P = ascPDRh.P = guiIn.hip_pos_kp;
    ascPDLh.D = ascPDRh.D = guiIn.hip_pos_kd;

    // Debug
    isManualFlightLegTO = guiIn.flight_to;
    isManualFlightLegTD = guiIn.flight_td;
    guiOut.walking_state = walkingState;
    guiOut.td_position = positionTD;
    guiOut.td_force = forceTD;
    guiOut.to_force = forceTO;
    guiOut.isEnabled = isEnabled();
}

/**
 * @brief Hip position tracking controller.
 * 
 * This function handles all hip motor commands independent of other
 * functions. It works by computing the inverse kinematics of the robot
 * selecting hip angles that result in the desired toe positions. This keeps
 * knee torques to a minimum.
 */
void ATCSlipWalking::hipController() {
    // Set hip controller toe positions
    toePosition.left = guiIn.left_toe_pos;
    toePosition.right = guiIn.right_toe_pos;

    // Compute inverse kinematics to keep lateral knee torque to a minimum
    std::tie(qLh, qRh) = ascHipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);

    // Rate limit motor velocities to smooth step inputs
    qLh = ascRateLimitLh(qLh, hipRateLimit);
    qRh = ascRateLimitRh(qRh, hipRateLimit);

    // Compute and set motor currents from position based PD controllers
    co.lLeg.motorCurrentHip = ascPDLh(qLh, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentHip = ascPDRh(qRh, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);
}

/**
 * @brief Constant position standing controller.
 * 
 * This function uses position control on the leg motors to allow the
 * robot to stand with the torso locked. Does not work with unlocked torso.
 */
// TODO stand up starting from rest on ground
void ATCSlipWalking::standingController() {
    // Compute motor angles
    std::tie(qmSA, qmSB) = ascCommonToolkit.legPos2MotorPos(qtSl, r0);
    std::tie(qmFA, qmFB) = ascCommonToolkit.legPos2MotorPos(qtFl, r0);

    // Rate limit motor velocities
    qmSA = ascRateLimitLmA(qmSA, legRateLimit);
    qmSB = ascRateLimitLmB(qmSB, legRateLimit);
    qmFA = ascRateLimitRmA(qmFA, legRateLimit);
    qmFB = ascRateLimitRmB(qmFB, legRateLimit);

    // Compute and set motor currents
    co.lLeg.motorCurrentA = ascPDLmA(qmSA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
    co.lLeg.motorCurrentB = ascPDLmB(qmSB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
    co.rLeg.motorCurrentA = ascPDRmA(qmFA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
    co.rLeg.motorCurrentB = ascPDRmB(qmFB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
}

/**
 * @brief Soft robot shutdown controller.
 * 
 * This function imposes virtual dampers on each motor allowing the
 * robot to safely and slowly shutdown.
 */
void ATCSlipWalking::shutdownController() {
    // Compute and set motor currents (applies virtual dampers to all actuators)
    co.lLeg.motorCurrentA = ascPDLmA(0.0, 0.0, 0.0, rs.lLeg.halfA.motorVelocity);
    co.lLeg.motorCurrentB = ascPDLmB(0.0, 0.0, 0.0, rs.lLeg.halfB.motorVelocity);
    co.lLeg.motorCurrentHip = ascPDLh(0.0, 0.0, 0.0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentA = ascPDRmA(0.0, 0.0, 0.0, rs.rLeg.halfA.motorVelocity);
    co.rLeg.motorCurrentB = ascPDRmB(0.0, 0.0, 0.0, rs.rLeg.halfB.motorVelocity);
    co.rLeg.motorCurrentHip = ascPDRh(0.0, 0.0, 0.0, rs.rLeg.hip.legBodyVelocity);
}

/**
 * @brief Stance leg force tracking controller.
 * @param rsSl Stance leg robot state pointer.
 * @param coSl Stance leg controller ouput pointer.
 * @param ascLegForceSl Stance leg PID force controller pointer.
 * @param ascRateLimitSr0 Rest spring length rate limiter pointer.
 * 
 * A simple stance phase controller simulating a virtual spring
 * between the hip and toe. Uses a force controller to then track these
 * forces that are based on leg deflection.
 */
void ATCSlipWalking::stanceController(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::controller_output_leg *coSl, ASCLegForce *ascLegForceSl, ASCRateLimit *ascRateLimitSr0) {
    // Rate limit change in spring rest length from current to desired
    r0Sl = ascRateLimitSr0->operator()(r0, springRateLimit);

    // Compute current leg angle and length
    std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    std::tie(dqSl, drSl) = ascCommonToolkit.motorVel2LegVel(rsSl->halfA.legAngle, rsSl->halfB.legAngle, rsSl->halfA.legVelocity, rsSl->halfB.legVelocity);

    // Compute current ATRIAS non-linear spring constant for given leg configuration
    std::tie(k, dk) = ascCommonToolkit.legStiffness(rSl, drSl, r0Sl);
    dk = 0.0; // TODO remove to verify noise issue

    // Define component forces and their derivatives
    forceSl.fx = -k*(rSl - r0Sl)*cos(qSl);
    forceSl.dfx = dk*cos(qSl)*(r0Sl - rSl) - drSl*cos(qSl)*k + dqSl*sin(qSl)*k*(rSl - r0Sl);
    forceSl.fz = k*(rSl - r0Sl)*sin(qSl);
    forceSl.dfz = drSl*sin(qSl)*k - dk*sin(qSl)*(r0Sl - rSl) + dqSl*cos(qSl)*k*(rSl - r0Sl);

    // Use force tracking controller to compute required motor currents
    std::tie(coSl->motorCurrentA, coSl->motorCurrentB) = ascLegForceSl->control(forceSl, *rsSl, rs.position);
}

/**
 * @brief Flight leg swing position trajectory tracking controller.
 * @param rsSl Stance leg robot state pointer.
 * @param rsFl Flight leg robot state pointer.
 * @param coFl Flight leg controller output pointer.
 * @param ascPDmA Flight leg motor A PD position controller pointer.
 * @param ascPDmB Flight leg motor B PD position controller pointer.
 * 
 * A position controller that tracks a cubic spline trajectory from
 * the exit conditions to the desired touchdown conditions. The touchdown
 * conditions are based on a simulated SLIP model walking gait.
 */
// TODO: Should make angles relative to world not to body (add in body pitch)
void ATCSlipWalking::legSwingController(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::robot_state_leg *rsFl, atrias_msgs::controller_output_leg *coFl, ASCPD *ascPDmA, ASCPD *ascPDmB) {
    // Compute the current leg angles and lengths and velocities
    std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    std::tie(dqSl, drSl) = ascCommonToolkit.motorVel2LegVel(rsSl->halfA.legAngle, rsSl->halfB.legAngle, rsSl->halfA.legVelocity, rsSl->halfB.legVelocity);
    std::tie(qFl, rFl) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.legAngle, rsFl->halfB.legAngle);
    std::tie(dqFl, drFl) = ascCommonToolkit.motorVel2LegVel(rsFl->halfA.legAngle, rsFl->halfB.legAngle, rsFl->halfA.legVelocity, rsFl->halfB.legVelocity);

    // Error catch the dependant to avoid trajectory being flipped if master leg starts past its predicted end location
    if (qeSl > qtSl - 0.2) {
        qtSl = qeSl + 0.2;
    }

    // Use a cubic spline interpolation to slave the flight leg angle to the stance leg angle
    dqeFl = 0.0;
    dqtFl = 0.3/(qtSl - qeSl);
    std::tie(ql, dql) = ascInterpolation.cubic(qeSl, qtSl, qeFl, qtFl, dqeFl, dqtFl, qSl, dqSl);

    // Use two cubic splines slaved to stance leg angle to retract and then extend the flight leg
    rtFl = r0 - swingLegRetraction;
    dreFl = 0.0;
    drtFl = 0.0;

    // Piece-wise cubic spline to slave the flight leg length to the stance leg angle
    if (qSl <= (qeSl + qtSl)/2.0) {
        // Leg retraction during first half
        std::tie(rl, drl) = ascInterpolation.cubic(qeSl, (qeSl + qtSl)/2.0, reFl, rtFl, dreFl, 0.0, qSl, dqSl);

    } else {
        // Leg extension during the second half
        std::tie(rl, drl) = ascInterpolation.cubic((qeSl + qtSl)/2.0, qtSl, rtFl, r0, 0.0, drtFl, qSl, dqSl);
    }

    // Convert desired leg angles and lengths into motor positions and velocities
    std::tie(qmFA, qmFB) = ascCommonToolkit.legPos2MotorPos(ql, rl);
    std::tie(dqmFA, dqmFB) = ascCommonToolkit.legVel2MotorVel(rl, dql, drl);

    // Compute and set motor currents from position based PD controllers
    coFl->motorCurrentA = ascPDmA->operator()(qmFA, rsFl->halfA.motorAngle, dqmFA, rsFl->halfA.motorVelocity);
    coFl->motorCurrentB = ascPDmB->operator()(qmFB, rsFl->halfB.motorAngle, dqmFB, rsFl->halfB.motorVelocity);

    // Clamp the motor currents during leg swing to let the amlifiers recover for the upcoming stance.
    //coFl->motorCurrentA = clamp(coFl->motorCurrentA, -30.0, 30.0);
    //coFl->motorCurrentB = clamp(coFl->motorCurrentB, -30.0, 30.0);
}

/**
 * @brief Single support event triggers.
 * @param rsSl Stance leg robot state pointer.
 * @param rsFl Flight leg robot state pointer.
 * @param ascLegForceSl Stance leg PID force controller pointer.
 * @param ascLegForceFl Flight leg PID force controller pointer.
 * @param ascRateLimitSr0 Stance rest spring length rate limiter pointer.
 * @param ascRateLimitFr0 Flight rest spring length rate limiter pointer.
 * 
 * This function computes logical conditionals and uses a decision tree
 * to determine if a single support event has been triggered and responds accordingly.
 */
// TODO compute conditionals with hip angle and body pitch accounted for
void ATCSlipWalking::singleSupportEvents(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::robot_state_leg *rsFl, ASCLegForce *ascLegForceSl, ASCLegForce *ascLegForceFl, ASCRateLimit *ascRateLimitSr0, ASCRateLimit *ascRateLimitFr0) {
    // Compute the current flight leg angle and length
    std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    std::tie(qFl, rFl) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.legAngle, rsFl->halfB.legAngle);

    // Compute actual leg force from spring deflection and robot state
    forceFl = ascLegForceFl->compute(*rsFl, rs.position);
    forceSl = ascLegForceSl->compute(*rsSl, rs.position);

    // Compute conditionals for event triggers
    isStanceLegTO = (forceSl.fz >= -forceThresholdTO);
    isFlightLegTD = (forceFl.fz <= -forceThresholdTD) && (rFl*sin(qFl) >= rSl*sin(qSl) - positionThresholdTD);
    isForwardStep = (rSl*cos(qSl) <= rFl*cos(qFl));
    isBackwardStep = (rSl*cos(qSl) > rFl*cos(qFl));

    // Set debug status values
    forceTD = forceFl.fz;
    forceTO = 0.0;
    positionTD = rSl*sin(qSl) - rFl*sin(qFl);

    // Flight leg touch down event (trigger next state)
    if ((isFlightLegTD || isManualFlightLegTD) && isForwardStep) {
        // Advance the walking state machine 1 step and loop to beginning if needed
        walkingState = (walkingState + 1) % 4;

        // Save the stance and flight leg exit conditions
        updateExitConditions(rsSl, rsFl, ascRateLimitSr0, ascRateLimitFr0);

    } else if ((isFlightLegTD || isManualFlightLegTD) && isBackwardStep) {
        // Do nothing, this means we started going backwards
    }

    // Stance leg take off
    if (isStanceLegTO) {
        // Do nothing, this would mean the robot is ballistic
    }
}


/**
 * @brief Double support event triggers.
 * @param rsSl Stance leg robot state pointer.
 * @param rsFl Flight leg robot state pointer.
 * @param ascLegForceSl Stance leg PID force controller pointer.
 * @param ascLegForceFl Flight leg PID force controller pointer.
 * @param ascRateLimitSr0 Stance rest spring length rate limiter pointer.
 * @param ascRateLimitFr0 Flight rest spring length rate limiter pointer.
 * 
 * This function computes logical conditionals and uses a decision tree
 * to determine if a double support event has been triggered and responds accordingly.
 */
void ATCSlipWalking::doubleSupportEvents(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::robot_state_leg *rsFl, ASCLegForce *ascLegForceSl, ASCLegForce *ascLegForceFl, ASCRateLimit *ascRateLimitSr0, ASCRateLimit *ascRateLimitFr0) {
    // Compute the current flight leg angle and length
    std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    std::tie(qFl, rFl) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.legAngle, rsFl->halfB.legAngle);

    // Compute actual leg force from spring deflection and robot state
    forceFl = ascLegForceFl->compute(*rsFl, rs.position);
    forceSl = ascLegForceSl->compute(*rsSl, rs.position);

    // Set debug status output values
    forceTD = 0.0;
    forceTO = forceFl.fz;
    positionTD = 0.0;

    // Compute conditionals for event triggers
    isFlightLegTO = forceFl.fz >= forceThresholdTO;
    isStanceLegTO = forceSl.fz >= forceThresholdTO;

    // Flight leg take off (trigger next state)
    if (isFlightLegTO || isManualFlightLegTO) {
        // Advance the walking state machine 1 step and loop to beginning if needed
        walkingState = (walkingState + 1) % 4;

        // Save the stance and flight leg exit conditions
        updateExitConditions(rsSl, rsFl, ascRateLimitSr0, ascRateLimitFr0);
    }

    // Stance leg take off
    if (isStanceLegTO) {
        // Do nothing, this means we started going backwards
    }
}

/**
 * @brief Update exit conditions.
 * @param rsSl Stance leg robot state pointer.
 * @param rsFl Flight leg robot state pointer.
 * @param ascRateLimitSr0 Stance rest spring length rate limiter pointer.
 * @param ascRateLimitFr0 Flight rest spring length rate limiter pointer.
 * 
 * Updates the exit conditions upon dynamic state transitions.
 */
void ATCSlipWalking::updateExitConditions(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::robot_state_leg *rsFl, ASCRateLimit *ascRateLimitSr0, ASCRateLimit *ascRateLimitFr0) {
    // Save the stance and flight leg exit conditions for use in leg swing trajectory generation
    std::tie(qeFl, reFl) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.legAngle, rsFl->halfB.legAngle);
    std::tie(dqeFl, dreFl) = ascCommonToolkit.motorVel2LegVel(rsFl->halfA.legAngle, rsFl->halfB.legAngle, rsFl->halfA.legVelocity, rsFl->halfB.legVelocity);
    std::tie(qeSl, reSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    std::tie(dqeSl, dreSl) = ascCommonToolkit.motorVel2LegVel(rsSl->halfA.legAngle, rsSl->halfB.legAngle, rsSl->halfA.legVelocity, rsSl->halfB.legVelocity);

    // Compute current rest leg angle and length
    std::tie(qeFm, reFm) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.motorAngle, rsFl->halfB.motorAngle);
    std::tie(qeSm, reSm) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.motorAngle, rsSl->halfB.motorAngle);

    // Reset rate limiters
    ascRateLimitSr0->reset(reSm);
    ascRateLimitFr0->reset(reFm);
}


ORO_CREATE_COMPONENT(ATCSlipWalking)

}
}
