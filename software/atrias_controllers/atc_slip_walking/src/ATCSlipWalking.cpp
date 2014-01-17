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
    ascRateLimitRh(this, "ascRateLimitRh"),
    ascRateLimitLr0(this, "ascRateLimitLr0"),
    ascRateLimitRr0(this, "ascRateLimitRr0")
{
    // Startup is handled by the ATC class
    setStartupEnabled(true);

    // Set hard coded rate limits
    legRateLimit = 0.5; // [rad/s]
    hipRateLimit = 0.5; // [rad/s]
    springRateLimit = 0.1; // [m/s]

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
    
    // Run safety checks and trigger E-stop if needed
    checkSafeties();

    // Run the hip controller
    hipController();

    // Main controller state machine
    switch (controllerState) {
        case 0: // Stand upright in place
            // Save the stance and flight leg exit conditions (right = stance, left = flight)
            updateExitConditions(&rs.rLeg, &rs.lLeg, &ascRateLimitRr0, &ascRateLimitLr0);

            // Reset walking state parameters
            walkingState = 1; // Double support

            // Call standing controller
            standingController();
            break;

        case 1: // SLIP  walking
            // SLIP  walking controller state machine
            switch (walkingState) {
                case 0: // Right leg single support (right = stance, left = flight)
                    legSwingController(&rs.rLeg, &rs.lLeg, &co.lLeg, &ascPDLmA, &ascPDLmB);
                    //stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR, &ascRateLimitRr0);
                    passiveStanceController(&rs.rLeg, &co.rLeg, &ascPDRmA, &ascPDRmB, &ascRateLimitRr0);
                    singleSupportEvents(&rs.rLeg, &rs.lLeg, &ascLegForceR, &ascLegForceL, &ascRateLimitRr0, &ascRateLimitLr0);
                    break;

                case 1: // Double support (right = flight, left = stance)
                    //stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR, &ascRateLimitRr0);
                    //stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL, &ascRateLimitLr0);
                    passiveStanceController(&rs.rLeg, &co.rLeg, &ascPDRmA, &ascPDRmB, &ascRateLimitRr0);
                    passiveStanceController(&rs.lLeg, &co.lLeg, &ascPDLmA, &ascPDLmB, &ascRateLimitLr0);
                    doubleSupportEvents(&rs.lLeg, &rs.rLeg, &ascLegForceL, &ascLegForceR, &ascRateLimitLr0, &ascRateLimitRr0);
                    break;

                case 2: // Left leg single support (right = flight, left = stance)
                    legSwingController(&rs.lLeg, &rs.rLeg, &co.rLeg, &ascPDRmA, &ascPDRmB);
                    //stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL, &ascRateLimitLr0);
                    passiveStanceController(&rs.lLeg, &co.lLeg, &ascPDLmA, &ascPDLmB, &ascRateLimitLr0);
                    singleSupportEvents(&rs.lLeg, &rs.rLeg, &ascLegForceL, &ascLegForceR, &ascRateLimitLr0, &ascRateLimitRr0);
                    break;

                case 3: // Double support (right = stance, left = flight)
                    //stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL, &ascRateLimitLr0);
                    //stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR, &ascRateLimitRr0);
                    passiveStanceController(&rs.lLeg, &co.lLeg, &ascPDLmA, &ascPDLmB, &ascRateLimitLr0);
                    passiveStanceController(&rs.rLeg, &co.rLeg, &ascPDRmA, &ascPDRmB, &ascRateLimitRr0);
                    doubleSupportEvents(&rs.rLeg, &rs.lLeg, &ascLegForceR, &ascLegForceL, &ascRateLimitRr0, &ascRateLimitLr0);
                    break;
            }
            break;

        case 2: // Shutdown
            // Call shutdown controller
            shutdownController();

            // Reset walking state parameters
            walkingState = 0; // Right leg single support

            // Save the stance and flight leg exit conditions (right = stance, left = flight)
            updateExitConditions(&rs.rLeg, &rs.lLeg, &ascRateLimitRr0, &ascRateLimitLr0);
            break;
    }

    // Limit motor currents to GUI specified value
    co.lLeg.motorCurrentA = clamp(co.lLeg.motorCurrentA, -currentLimit, currentLimit);
    co.lLeg.motorCurrentB = clamp(co.lLeg.motorCurrentB, -currentLimit, currentLimit);
    co.rLeg.motorCurrentA = clamp(co.rLeg.motorCurrentA, -currentLimit, currentLimit);
    co.rLeg.motorCurrentB = clamp(co.rLeg.motorCurrentB, -currentLimit, currentLimit);

} // controller

/**
 * @brief Run safety checks.
 * 
 * This function performs various safety checks and triggers a software
 * emergency stop if needed.
 */
void ATCSlipWalking::checkSafeties() {
    // Trigger an E-stop if a motor velocity goes over the hardcoded limit
    if (abs(rs.lLeg.halfA.rotorVelocity) > velocityLimit ||
        abs(rs.lLeg.halfB.rotorVelocity) > velocityLimit ||
        abs(rs.rLeg.halfA.rotorVelocity) > velocityLimit ||
        abs(rs.rLeg.halfB.rotorVelocity) > velocityLimit ||
        abs(rs.lLeg.hip.legBodyVelocity) > velocityLimit ||
        abs(rs.rLeg.hip.legBodyVelocity) > velocityLimit)
    {
        // Trigger E-stop
        printf("Software E-Stop triggered by motor velocity limit check.\n");
        commandEStop();
    }

    // Trigger an E-stop if a spring deflection goes over the hard coded limit
    if (abs(rs.lLeg.halfA.motorAngle - rs.lLeg.halfA.legAngle) > deflectionLimit ||
        abs(rs.lLeg.halfB.motorAngle - rs.lLeg.halfB.legAngle) > deflectionLimit ||
        abs(rs.rLeg.halfA.motorAngle - rs.rLeg.halfA.legAngle) > deflectionLimit ||
        abs(rs.rLeg.halfB.motorAngle - rs.rLeg.halfB.legAngle) > deflectionLimit)
    {
        // Trigger E-stop
        printf("Software E-Stop triggered by spring deflection limit check.\n");
        commandEStop();
    }
} // checkSafeties

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
    switchMethod = guiIn.switch_method;

    // Gait options
    swingLegRetraction = guiIn.swing_leg_retraction;
    stanceLegExtension = guiIn.stance_leg_extension;
    r0 = guiIn.leg_length;
    torsoAngle = guiIn.torso_angle;
    q1 = guiIn.q1;
    q2 = guiIn.q2;
    q3 = guiIn.q3;
    q4 = guiIn.q4;

    // Leg gains
    ascPDLmA.P = ascPDLmB.P = ascPDRmA.P = ascPDRmB.P = guiIn.leg_pos_kp;
    ascPDLmA.D = ascPDLmB.D = ascPDRmA.D = ascPDRmB.D = guiIn.leg_pos_kd;
    ascLegForceL.kp = ascLegForceR.kp = guiIn.leg_for_kp;
    ascLegForceL.ki = ascLegForceR.ki = guiIn.leg_for_ki;
    ascLegForceL.kd = ascLegForceR.kd = guiIn.leg_for_kd;

    // Hip gains
    ascPDLh.P = ascPDRh.P = guiIn.hip_pos_kp;
    ascPDLh.D = ascPDRh.D = guiIn.hip_pos_kd;

    // Debug buttons and outputs
    currentLimit = guiIn.current_limit;
    velocityLimit = guiIn.velocity_limit;
    deflectionLimit = guiIn.deflection_limit;
    isManualSwingLegTO = guiIn.flight_to;
    isManualSwingLegTD = guiIn.flight_td;
    guiOut.walking_state = walkingState;

    // Return is enabled
    guiOut.isEnabled = isEnabled();
} // updateController

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
} // hipController

/**
 * @brief Constant position standing controller.
 * 
 * This function uses position control on the leg motors to allow the
 * robot to stand with the torso locked.
 */
void ATCSlipWalking::standingController() {
    // Compute target motor angles
    std::tie(qmSA, qmSB) = ascCommonToolkit.legPos2MotorPos(q3, r0);
    std::tie(qmFA, qmFB) = ascCommonToolkit.legPos2MotorPos(q1, r0);

    // Compute current torso states
    qb = rs.position.bodyPitch;
    dqb = rs.position.bodyPitchVelocity;

    // Negate body pitch
    qmSA -= qb - 3.0*M_PI/2.0;
    qmSB -= qb - 3.0*M_PI/2.0;
    qmFA -= qb - 3.0*M_PI/2.0;
    qmFB -= qb - 3.0*M_PI/2.0;

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
} // standingController

/**
 * @brief Soft robot shutdown controller.
 * 
 * This function imposes virtual dampers on each motor allowing the
 * robot to safely and slowly shutdown.
 */
void ATCSlipWalking::shutdownController() {
    // Compute and set motor currents (applies virtual dampers to all actuators)
    co.lLeg.motorCurrentA   = ascPDLmA(0.0, 0.0, 0.0, rs.lLeg.halfA.motorVelocity);
    co.lLeg.motorCurrentB   = ascPDLmB(0.0, 0.0, 0.0, rs.lLeg.halfB.motorVelocity);
    co.lLeg.motorCurrentHip = ascPDLh(0.0, 0.0, 0.0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentA   = ascPDRmA(0.0, 0.0, 0.0, rs.rLeg.halfA.motorVelocity);
    co.rLeg.motorCurrentB   = ascPDRmB(0.0, 0.0, 0.0, rs.rLeg.halfB.motorVelocity);
    co.rLeg.motorCurrentHip = ascPDRh(0.0, 0.0, 0.0, rs.rLeg.hip.legBodyVelocity);
} // shutdownController

/**
 * @brief Stance leg passive controller.
 * @param rsSl Stance leg robot state pointer.
 * @param coSl Stance leg controller ouput pointer.
 * @param ascPDSmA Stance leg motor A PD position controller pointer.
 * @param ascPDSmB Stance leg motor B PD position controller pointer.
 * 
 * A simple stance phase controller allowing only leg length 
 * forces with zero leg angle torques. Uses a position controller to 
 * keep virtual motor leg length constant while minimizing spring 
 * about the hip.
 */
void ATCSlipWalking::passiveStanceController(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::controller_output_leg *coSl, ASCPD *ascPDSmA, ASCPD *ascPDSmB, ASCRateLimit *ascRateLimitSr0) {
    // Rate limit change in spring rest length from current to desired
    r0Sl = ascRateLimitSr0->operator()(r0 + stanceLegExtension, springRateLimit);

    // Compute current stance leg states
    std::tie(ql, rl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    std::tie(dql, drl) = ascCommonToolkit.motorVel2LegVel(rsSl->halfA.legAngle, rsSl->halfB.legAngle, rsSl->halfA.legVelocity, rsSl->halfB.legVelocity);

    // Compute and set motor angles such that there is no hip torque, only
    // axial leg forces
    std::tie(qmSA, qmSB) = ascCommonToolkit.legPos2MotorPos(ql, r0Sl);
    std::tie(dqmSA, dqmSB) = ascCommonToolkit.legVel2MotorVel(r0Sl, dql, 0.0);

    // Compute and set motor currents from position based PD controllers
    coSl->motorCurrentA = ascPDSmA->operator()(qmSA, rsSl->halfA.motorAngle, dqmSA, rsSl->halfA.motorVelocity);
    coSl->motorCurrentB = ascPDSmB->operator()(qmSB, rsSl->halfB.motorAngle, dqmSB, rsSl->halfB.motorVelocity);
} // passiveStanceController

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

    // Compute current ATRIAS non-linear spring force for given leg configuration
    std::tie(fa, dfa) = ascCommonToolkit.legForce(rSl, drSl, r0Sl);

    // Define component forces and their derivatives
    forceSl.fx = fa*cos(qSl);
    forceSl.dfx = -dqSl*sin(qSl)*fa;// + dfa*cos(qSl);
    forceSl.fz = -fa*sin(qSl);
    forceSl.dfz = -dqSl*cos(qSl)*fa;// - dfa*sin(qSl);

    // Use force tracking controller to compute required motor currents
    // Force controller has built into world coordinate conversion
    std::tie(coSl->motorCurrentA, coSl->motorCurrentB) = ascLegForceSl->control(forceSl, *rsSl, rs.position);
} // stanceController

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
void ATCSlipWalking::legSwingController(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::robot_state_leg *rsFl, atrias_msgs::controller_output_leg *coFl, ASCPD *ascPDmA, ASCPD *ascPDmB) {
    // Compute current stance leg states
    std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    std::tie(dqSl, drSl) = ascCommonToolkit.motorVel2LegVel(rsSl->halfA.legAngle, rsSl->halfB.legAngle, rsSl->halfA.legVelocity, rsSl->halfB.legVelocity);

    // Compute current flight leg states
    std::tie(qFl, rFl) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.legAngle, rsFl->halfB.legAngle);
    std::tie(dqFl, drFl) = ascCommonToolkit.motorVel2LegVel(rsFl->halfA.legAngle, rsFl->halfB.legAngle, rsFl->halfA.legVelocity, rsFl->halfB.legVelocity);

    // Compute current torso states
    qb = rs.position.bodyPitch;
    dqb = rs.position. bodyPitchVelocity;

    // Convert leg angle and velocities to world coordinates
    qSl  += qb - 3.0*M_PI/2.0;
    dqSl += dqb;
    qFl  += qb - 3.0*M_PI/2.0;
    dqFl += dqb;

    // Compute gait parameter and velocity
    s = (qSl - q2)/(q3 - q2);
    ds = dqSl/(q3 - q2);

    // Limit gait parameter between zero and one
    s = clamp(s, 0.0, 1.0);

    // Use a cubic spline interpolation to slave the flight leg angle to the stance leg angle
    std::tie(qm, dqm) = ascInterpolation.cubic(0.0, 1.0, qeFm, q1, 0.0, 0.0, s, ds);

    // Compute leg retraction target length
    rtFm = r0 - swingLegRetraction;

    // Piece-wise cubic spline to slave the flight leg length to the stance leg angle
    if (s <= 0.5) {
        // Leg retraction during first half
        std::tie(rm, drm) = ascInterpolation.cubic(0.0, 0.5, reFm, rtFm, 0.0, 0.0, s, ds);
    } else {
        // Leg extension during the second half
        std::tie(rm, drm) = ascInterpolation.cubic(0.5, 1.0, rtFm, r0, 0.0, 0.0, s, ds);
    }

    // Convert desired leg angles and lengths into motor positions and velocities
    std::tie(qmFA, qmFB) = ascCommonToolkit.legPos2MotorPos(qm, rm);
    std::tie(dqmFA, dqmFB) = ascCommonToolkit.legVel2MotorVel(rm, dqm, drm);

    // Convert leg angle and velocities to relative coordinates
    qmFA  -= qb - 3.0*M_PI/2.0;
    dqmFA -= dqb;
    qmFB  -= qb - 3.0*M_PI/2.0;
    dqmFB -= dqb;

    // Compute and set motor currents from position based PD controllers
    coFl->motorCurrentA = ascPDmA->operator()(qmFA, rsFl->halfA.motorAngle, dqmFA, rsFl->halfA.motorVelocity);
    coFl->motorCurrentB = ascPDmB->operator()(qmFB, rsFl->halfB.motorAngle, dqmFB, rsFl->halfB.motorVelocity);
} // legSwingController

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
void ATCSlipWalking::singleSupportEvents(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::robot_state_leg *rsFl, ASCLegForce *ascLegForceSl, ASCLegForce *ascLegForceFl, ASCRateLimit *ascRateLimitSr0, ASCRateLimit *ascRateLimitFr0) {
    // Compute current stance leg states
    std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    
    // Compute current torso states
    qb = rs.position.bodyPitch;
    
    // Convert leg angle
    // TODO: Convert to what?
    qSl += 3.0*M_PI/2.0 - qb;

    // Make sure we step forward and dont trigger next state if we back step
    isForwardStep = (qSl >= ((q2 + q3)/2.0));

    // Handle different trigger methods
    switch (switchMethod) {
        case 0: // Contact sensing + automatic switch
            isTrigger = rsFl->onGround || (qSL >= q3);
            break;

        case 1: // Automatic switch based on gait parameter
            isTrigger = (qSl >= q3);
            break;
    }

    // Flight leg touch down event (trigger next state)
    if ((isTrigger || isManualSwingLegTD) && isForwardStep) {
        // Advance the walking state machine 1 step and loop to beginning if needed
        walkingState = (walkingState + 1) % 4;

        // Save the stance and flight leg exit conditions
        updateExitConditions(rsSl, rsFl, ascRateLimitSr0, ascRateLimitFr0);
    }
} // singleSupportEvents

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
    // Compute current stance leg states
    std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    
    // Compute current torso states
    qb = rs.position.bodyPitch;
    
    // Convert leg angle
    qSl += 3.0*M_PI/2.0 - qb;
    
    // Handle different trigger methods
    switch (switchMethod) {
        case 0: // Contact sensing + automatic trigger
            isTrigger = !rsFl->onGround || (qSl >= q2);
            break;

        case 1: // Automatic switch based on gait parameter
            isTrigger = (qSl >= q2);
            break;
    }

    // Flight leg take off (trigger next state)
    if (isTrigger || isManualSwingLegTO) {
        // Advance the walking state machine 1 step and loop to beginning if needed
        walkingState = (walkingState + 1) % 4;

        // Save the stance and flight leg exit conditions
        updateExitConditions(rsSl, rsFl, ascRateLimitSr0, ascRateLimitFr0);
    }
} // doubleSupportEvents

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
    // Compute current rest leg angle and length
    std::tie(qeFm, reFm) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.motorAngle, rsFl->halfB.motorAngle);
    std::tie(qeSm, reSm) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.motorAngle, rsSl->halfB.motorAngle);

    // Reset rate limiters
    ascRateLimitSr0->reset(reSm);
    ascRateLimitFr0->reset(reFm);
} // updateExitConditions

ORO_CREATE_COMPONENT(ATCSlipWalking)

} // controller
} // atrias
