/**
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
 * @brief This is the main function for the top-level controller.
 * The ATC class automatically handles startup and shutdown,
 * if they are not disabled.
 */
void ATCSlipWalking::controller() {
    // Update current robot state
    updateState();

    // Run hip controller
    hipController();

    // Main controller state machine
    switch (controllerState) {
        case 0: // Stand upright in place
            // Call standing controller
            standingController();
            // Reset walking state parameters
            walkingState = 0;
            // Save the stance and flight leg exit conditions
            updateExitConditions(&rs.rLeg, &rs.lLeg, &ascRateLimitRr0, &ascRateLimitLr0); // (stance leg, flight leg)
            break;

        case 1: // SLIP  walking
            // SLIP  walking controller state machine
            switch (walkingState) {
                case 0: // Right leg single support (right = stance, left = flight)
                    // Define horizontal pushoff force
                    pushoffForce = guiIn.pushoff_force;
                    // Run state specific controller and event functions
                    stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR, &ascRateLimitRr0);
                    legSwingController(&rs.rLeg, &rs.lLeg, &co.lLeg, &ascPDLmA, &ascPDLmB);
                    singleSupportEvents(&rs.rLeg, &rs.lLeg, &ascLegForceR, &ascLegForceL, &ascRateLimitRr0, &ascRateLimitLr0);
                    /*if (guiIn.apply_hip_torque && ~hipTorqueFlag) {
                        co.rLeg.motorCurrentA += guiIn.hip_torque;
                        co.rLeg.motorCurrentB += guiIn.hip_torque;
                    }*/
                    break;

                case 1: // Double support (right = flight, left = stance)
                    // Define horizontal pushoff force
                    pushoffForce = 0.0;
                    // Run state specific controller and event functions
                    stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR, &ascRateLimitRr0);
                    stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL, &ascRateLimitLr0);
                    doubleSupportEvents(&rs.lLeg, &rs.rLeg, &ascLegForceL, &ascLegForceR, &ascRateLimitLr0, &ascRateLimitRr0);
                    break;

                case 2: // Left leg single support (right = flight, left = stance)
                    // Define horizontal pushoff force
                    pushoffForce = guiIn.pushoff_force;
                    // Run state specific controller and event functions
                    stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL, &ascRateLimitLr0);
                    legSwingController(&rs.lLeg, &rs.rLeg, &co.rLeg, &ascPDRmA, &ascPDRmB);
                    singleSupportEvents(&rs.lLeg, &rs.rLeg, &ascLegForceL, &ascLegForceR, &ascRateLimitLr0, &ascRateLimitRr0);
                    /*if (guiIn.apply_hip_torque && ~hipTorqueFlag) {
                        co.lLeg.motorCurrentA += guiIn.hip_torque;
                        co.lLeg.motorCurrentB += guiIn.hip_torque;
                    }*/
                    break;

                case 3: // Double support (right = stance, left = flight)
                    // Define horizontal pushoff force
                    pushoffForce = 0.0;
                    // Run state specific controller and event functions
                    stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL, &ascRateLimitLr0);
                    stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR, &ascRateLimitRr0);
                    doubleSupportEvents(&rs.rLeg, &rs.lLeg, &ascLegForceR, &ascLegForceL, &ascRateLimitRr0, &ascRateLimitLr0);
                    break;
            }
            break;

        case 2: // Shutdown
            // Call shutdown controller
            shutdownController();
            // Reset walking state parameters
            walkingState = 0;
            // Save the stance and flight leg exit conditions
            updateExitConditions(&rs.rLeg, &rs.lLeg, &ascRateLimitRr0, &ascRateLimitLr0); // (stance leg, flight leg)
            break;
    }
}

/**
 * @brief This function handles all of the non-controller related updating.
 */
 /**
 * @brief This function handles all communication to and from the GUI.
 */
void ATCSlipWalking::updateState() {
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

    // Get GUI inputs
    controllerState = guiIn.main_controller;

    // Update GUI walking state display
    guiOut.isEnabled = isEnabled();
    guiOut.walking_state = walkingState;
    guiOut.td_force = forceTD;
    guiOut.to_force = forceTO;
    guiOut.td_position = positionTD;
    //hipTorqueFlag = false;

    // Event and state trigger parameters
    isManualFlightLegTO = guiIn.flight_to; // Manually triger events
    isManualFlightLegTD = guiIn.flight_td; // Manually triger events
    forceThresholdTO = guiIn.force_threshold_td;
    forceThresholdTO = guiIn.force_threshold_to;
    positionThresholdTD = guiIn.position_threshold_td;

    // Gait definitions
    r0 = guiIn.slip_leg; // Rest spring length
    ascCommonToolkit.ks = guiIn.atrias_spring; // Rotational spring constant for ATRIAS
    swingLegRetraction = guiIn.swing_leg_retraction; // Amount the leg retracts in swing phase
    qtSl = guiIn.stance_leg_target; // Predicted stance leg angle at flight leg TD
    qtFl = guiIn.flight_leg_target; // Target flight leg angle at TD

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
}

/**
 * @brief This function handles all hip motor commands independent of other
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
 * @brief This functions uses position control on the leg motors to allow the
 * robot to stand with the torso locked. Does not work with unlocked torso.
 */
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
 * @brief This function imposes virtual dampers on each motor allowing the
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
 * @brief A simple stance phase controller simulating a virtual spring
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
    dk = 0.0; // TODO: set to zero for debugging to track down noise.

    // Define component forces and their derivatives
    forceSl.fx = -k*(rSl - r0Sl)*cos(qSl);
    forceSl.dfx = dk*cos(qSl)*(r0Sl - rSl) - drSl*cos(qSl)*k + dqSl*sin(qSl)*k*(rSl - r0Sl);
    forceSl.fz = k*(rSl - r0Sl)*sin(qSl);
    forceSl.dfz = drSl*sin(qSl)*k - dk*sin(qSl)*(r0Sl - rSl) + dqSl*cos(qSl)*k*(rSl - r0Sl);

    // Add a horizontal force to inject energy and pushoff
    forceSl.fx += pushoffForce;
    
    // Use force tracking controller to compute required motor currents
    std::tie(coSl->motorCurrentA, coSl->motorCurrentB) = ascLegForceSl->control(forceSl, *rsSl, rs.position);
}

/**
 * @brief A position controller that tracks a cubic spline trajectory from 
 * the exit conditions to the desired touchdown conditions. The touchdown
 * conditions are based on a simulated SLIP model walking gait.
 */
void ATCSlipWalking::legSwingController(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::robot_state_leg *rsFl, atrias_msgs::controller_output_leg *coFl, ASCPD *ascPDmA, ASCPD *ascPDmB) {
    // A gait is defined by
        // The TO angle of the flight leg
        // The TO length of the flight leg (not as important, could use to inject energy)
        // The TD angle of the flight leg (after constants work could fit curve for various gaits vs dx)
        // The TD length of the flight leg (should be constant)
        // The angle of the stance leg when TO of flight leg occurs
        // The simulated angle of the stance leg during flight TD
        // Amount of flight leg retraction desired
        // Velocities of cubic splines can be adjusted as needed.

    // TODO: Should make angles relative to world not to body (add in body pitch)
    // TODO: If qeSl > qtSl then we have a issue, need to error catch but not sure what to do if it fails (maybe just take current value and add on some resonable amount)

    // Compute the current leg angles and lengths and velocities
    std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    std::tie(dqSl, drSl) = ascCommonToolkit.motorVel2LegVel(rsSl->halfA.legAngle, rsSl->halfB.legAngle, rsSl->halfA.legVelocity, rsSl->halfB.legVelocity);
    std::tie(qFl, rFl) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.legAngle, rsFl->halfB.legAngle);
    std::tie(dqFl, drFl) = ascCommonToolkit.motorVel2LegVel(rsFl->halfA.legAngle, rsFl->halfB.legAngle, rsFl->halfA.legVelocity, rsFl->halfB.legVelocity);

    // Error catch the dependant to avoid trajectory being flipped if master leg starts past its predicted end location
    if (qeSl > qtSl - 0.1) {
        qtSl = qeSl + 0.1;
    }

    // Set flag to disable constant hip torque debug toggle
    /*if (qSl >= qtSl) {
        hipTorqueFlag = true;
    }*/

    // Use a cubic spline interpolation to slave the flight leg angle and length to the stance leg angle
    dqeFl = 0.0;
    dqtFl = 0.3/(qtSl - qeSl);
    std::tie(ql, dql) = ascInterpolation.cubic(qeSl, qtSl, qeFl, qtFl, dqeFl, dqtFl, qSl, dqSl);

    // Use two cubic splines slaved to stance leg angle to retract and then extend the flight leg
    rtFl = r0 - swingLegRetraction;
    dreFl = 0.0;
    drtFl = 0.0;
    // Switch between cubic spline depending on value of domain
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
    coFl->motorCurrentA = clamp(coFl->motorCurrentA, -30.0, 30.0);
    coFl->motorCurrentB = clamp(coFl->motorCurrentB, -30.0, 30.0);
}

/**
 * @brief This function computes logical conditionals and uses a decision tree 
 * to determine if a single support event has been triggered and responds accordingly.
 */
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
        // Do nothing - Regress the walking state machine 1 step and loop to end if needed (the + 4 prevents negatives)
        //walkingState = (walkingState - 1 + 4) % 4;
    }

    // Stance leg take off
    if (isStanceLegTO) {
        // Do nothing, this would mean the robot is bouncing
    }
}


/**
 * @brief This function computes logical conditionals and uses a decision tree 
 * to determine if a double support event has been triggered and responds accordingly.
 */
void ATCSlipWalking::doubleSupportEvents(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::robot_state_leg *rsFl, ASCLegForce *ascLegForceSl, ASCLegForce *ascLegForceFl, ASCRateLimit *ascRateLimitSr0, ASCRateLimit *ascRateLimitFr0) {
    // Compute the current flight leg angle and length
    std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    std::tie(qFl, rFl) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.legAngle, rsFl->halfB.legAngle);

    // Compute actual leg force from spring deflection and robot state
    forceFl = ascLegForceFl->compute(*rsFl, rs.position);
    forceSl = ascLegForceSl->compute(*rsSl, rs.position);

    // Set debug status values
    forceTD = 0.0;
    forceTO = forceFl.fz;
    positionTD = 0.0;

    // Compute conditionals for event triggers
    isFlightLegTO = forceFl.fz >= -forceThresholdTO;
    isStanceLegTO = forceSl.fz >= -forceThresholdTO;

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
 * @brief Updates the exit conditions upon dynamic state transitions.
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
