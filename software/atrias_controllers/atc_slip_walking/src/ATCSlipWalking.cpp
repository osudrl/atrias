/**
 * @file ATCSlipWalking.cpp
 * @brief A Spring Loaded Inverted Pendulum (SLIP) template model based
 * walking controller.
 * @author Mikhail Jones and Andrew Peekema
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
    springRateLimit = 0.3; // [m/s]

    // Initialize walking state
    walkingState = 3;

    // Initialize the flight leg variable
    s = 0.0;
    sPrev = 0.0;

    // Initialize toe filter
    rFilteredToe.assign (120,5000.0);  // 120 doubles with a value of 5000
    lFilteredToe.assign (120,5000.0);
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

    // Update the toe filter
    updateToeFilter(rs.rLeg.toeSwitch, &rFilteredToe);
    updateToeFilter(rs.lLeg.toeSwitch, &lFilteredToe);

    // Run the hip controller
    hipController();

    // Main controller state machine
    switch (controllerState) {
        case 0: // Stand upright in place
            // Reset walking control variables (right = stance, left = flight)
            resetFlightLegParameters(&rs.lLeg, &ascRateLimitLr0);
            walkingState = 3; // Double support

            // Call standing controller
            standingController();
            break;

        case 1: // SLIP  walking
            // SLIP  walking controller state machine
            switch (walkingState) {
                case 0: // Right leg single support (right = stance, left = flight)
                    legSwingController(&rs.rLeg, &rs.lLeg, &co.lLeg, &ascPDLmA, &ascPDLmB);
                    stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR, &ascRateLimitRr0);
                    singleSupportEvents(&rs.rLeg, &rs.lLeg, &lFilteredToe);
                    break;

                case 1: // Double support (right = flight, left = stance)
                    stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR, &ascRateLimitRr0);
                    stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL, &ascRateLimitLr0);
                    doubleSupportEvents(&rs.lLeg, &rs.rLeg, &ascRateLimitRr0);
                    break;

                case 2: // Left leg single support (right = flight, left = stance)
                    legSwingController(&rs.lLeg, &rs.rLeg, &co.rLeg, &ascPDRmA, &ascPDRmB);
                    stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL, &ascRateLimitLr0);
                    singleSupportEvents(&rs.lLeg, &rs.rLeg, &rFilteredToe);
                    break;

                case 3: // Double support (right = stance, left = flight)
                    stanceController(&rs.lLeg, &co.lLeg, &ascLegForceL, &ascRateLimitLr0);
                    stanceController(&rs.rLeg, &co.rLeg, &ascLegForceR, &ascRateLimitRr0);
                    doubleSupportEvents(&rs.rLeg, &rs.lLeg, &ascRateLimitLr0);
                    break;
            }
            break;

        case 2: // Shutdown
            // Call shutdown controller
            shutdownController();

            // Reset walking control variables (right = stance, left = flight)
            resetFlightLegParameters(&rs.lLeg, &ascRateLimitLr0);
            walkingState = 3; // Double support
            break;
    }

    // Run safety checks and trigger E-stop if needed
    checkSafeties();

    // Make the gradients between the logged walking states unique
    if (walkingState == 2) {
        logOut.walkingState = 3;
    } else if (walkingState == 3) {
        logOut.walkingState = 6;
    } else {
        logOut.walkingState = walkingState;
    }

} // controller

/**
 * @brief Run safety checks.
 * 
 * This function performs various safety checks and triggers a software
 * emergency stop if needed.
 */
void ATCSlipWalking::checkSafeties() {
    // Limit motor currents to GUI specified value
    co.lLeg.motorCurrentA = clamp(co.lLeg.motorCurrentA, -currentLimit, currentLimit);
    co.lLeg.motorCurrentB = clamp(co.lLeg.motorCurrentB, -currentLimit, currentLimit);
    co.rLeg.motorCurrentA = clamp(co.rLeg.motorCurrentA, -currentLimit, currentLimit);
    co.rLeg.motorCurrentB = clamp(co.rLeg.motorCurrentB, -currentLimit, currentLimit);

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
        printf("GUI velocityLimit: %f\n", velocityLimit);
        commandEStop();
    }

    // Trigger an E-stop if a spring deflection goes over the hard coded limit
    if (abs(rs.lLeg.halfA.motorAngle - rs.lLeg.halfA.legAngle) > deflectionLimit ||
        abs(rs.lLeg.halfB.motorAngle - rs.lLeg.halfB.legAngle) > deflectionLimit ||
        abs(rs.rLeg.halfA.motorAngle - rs.rLeg.halfA.legAngle) > deflectionLimit ||
        abs(rs.rLeg.halfB.motorAngle - rs.rLeg.halfB.legAngle) > deflectionLimit)
    {
        // Trigger E-stop
        //printf("Software E-Stop triggered by spring deflection limit check.\n");
        printf("GUI deflectionLimit: %f\n", deflectionLimit);
        printf("Spring deflection (lA): %f\n", rs.lLeg.halfA.motorAngle - rs.lLeg.halfA.legAngle);
        printf("Spring deflection (rA): %f\n", rs.rLeg.halfA.motorAngle - rs.rLeg.halfA.legAngle);
        printf("Spring deflection (lB): %f\n", rs.lLeg.halfB.motorAngle - rs.lLeg.halfB.legAngle);
        printf("Spring deflection (rB): %f\n", rs.rLeg.halfB.motorAngle - rs.rLeg.halfB.legAngle);
        //commandEStop();
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

    // Torso options
    qvpp = guiIn.qvpp;
    rvpp = guiIn.rvpp;

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

    // Torso control
    double torsoCurrent = -(torsoAngle - qb)*guiIn.leg_pos_kp - (0.0 - dqb)*guiIn.leg_pos_kd;

    // Compute and set motor currents
    co.lLeg.motorCurrentA = ascPDLmA(qmSA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity) + torsoCurrent/2.0;
    co.lLeg.motorCurrentB = ascPDLmB(qmSB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity) + torsoCurrent/2.0;
    co.rLeg.motorCurrentA = ascPDRmA(qmFA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity) + torsoCurrent/2.0;
    co.rLeg.motorCurrentB = ascPDRmB(qmFB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity) + torsoCurrent/2.0;
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

    // Compute current stance leg states
    std::tie(qSl, rSl)   = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    std::tie(dqSl, drSl) = ascCommonToolkit.motorVel2LegVel(rsSl->halfA.legAngle, rsSl->halfB.legAngle, rsSl->halfA.legVelocity, rsSl->halfB.legVelocity);

    // Compute current torso states
    qb = rs.position.bodyPitch;
    dqb = rs.position.bodyPitchVelocity;

    // Convert leg angle and velocity to world coordinates
    qSl  += qb - 3.0*M_PI/2.0;
    dqSl += dqb;

    // Compute current ATRIAS non-linear spring force for given leg configuration
    std::tie(fa, dfa) = ascCommonToolkit.legForce(rSl, drSl, r0Sl);

    // Define component forces and their derivatives
    forceSl.fx  =  fa*cos(qSl);
    forceSl.dfx = -fa*sin(qSl)*dqSl + dfa*cos(qSl);
    forceSl.fz  = -fa*sin(qSl);
    forceSl.dfz = -fa*cos(qSl)*dqSl - dfa*sin(qSl);

    // Torso control
    // VPP control
    // Distance between leg pivot center and center of mass from ATRIAS solid model
    rcom = 0.12;
    // Angle between axial leg force and desired ground reaction force
    q = asin(rvpp*sin(qSl-qb+qvpp+asin(rcom*sin(qSl-qb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)))*1.0/sqrt(rSl*rSl+rcom*rcom+rvpp*rvpp-rSl*rcom*cos(qSl-qb)*2.0-rvpp*cos(qSl-qb+qvpp+asin(rcom*sin(qSl-qb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)))*sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)*2.0))+asin(rcom*sin(qSl-qb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0));
    dq = (rcom*cos(qSl-qb)*(dqSl-dqb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)-rcom*sin(qSl-qb)*1.0/pow(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0,3.0/2.0)*(drSl*rSl*2.0-drSl*rcom*cos(qSl-qb)*2.0+rSl*rcom*sin(qSl-qb)*(dqSl-dqb)*2.0)*(1.0/2.0))*1.0/sqrt(-((rcom*rcom)*pow(sin(qSl-qb),2.0))/(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)+1.0)+(rvpp*cos(qSl-qb+qvpp+asin(rcom*sin(qSl-qb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)))*(dqSl-dqb+(rcom*cos(qSl-qb)*(dqSl-dqb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)-rcom*sin(qSl-qb)*1.0/pow(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0,3.0/2.0)*(drSl*rSl*2.0-drSl*rcom*cos(qSl-qb)*2.0+rSl*rcom*sin(qSl-qb)*(dqSl-dqb)*2.0)*(1.0/2.0))*1.0/sqrt(-((rcom*rcom)*pow(sin(qSl-qb),2.0))/(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)+1.0))*1.0/sqrt(rSl*rSl+rcom*rcom+rvpp*rvpp-rSl*rcom*cos(qSl-qb)*2.0-rvpp*cos(qSl-qb+qvpp+asin(rcom*sin(qSl-qb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)))*sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)*2.0)-rvpp*sin(qSl-qb+qvpp+asin(rcom*sin(qSl-qb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)))*1.0/pow(rSl*rSl+rcom*rcom+rvpp*rvpp-rSl*rcom*cos(qSl-qb)*2.0-rvpp*cos(qSl-qb+qvpp+asin(rcom*sin(qSl-qb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)))*sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)*2.0,3.0/2.0)*(drSl*rSl*2.0-drSl*rcom*cos(qSl-qb)*2.0-rvpp*cos(qSl-qb+qvpp+asin(rcom*sin(qSl-qb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)))*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)*(drSl*rSl*2.0-drSl*rcom*cos(qSl-qb)*2.0+rSl*rcom*sin(qSl-qb)*(dqSl-dqb)*2.0)+rvpp*sin(qSl-qb+qvpp+asin(rcom*sin(qSl-qb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)))*sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)*(dqSl-dqb+(rcom*cos(qSl-qb)*(dqSl-dqb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)-rcom*sin(qSl-qb)*1.0/pow(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0,3.0/2.0)*(drSl*rSl*2.0-drSl*rcom*cos(qSl-qb)*2.0+rSl*rcom*sin(qSl-qb)*(dqSl-dqb)*2.0)*(1.0/2.0))*1.0/sqrt(-((rcom*rcom)*pow(sin(qSl-qb),2.0))/(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)+1.0))*2.0+rSl*rcom*sin(qSl-qb)*(dqSl-dqb)*2.0)*(1.0/2.0))*1.0/sqrt(-((rvpp*rvpp)*pow(sin(qSl-qb+qvpp+asin(rcom*sin(qSl-qb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0))),2.0))/(rSl*rSl+rcom*rcom+rvpp*rvpp-rSl*rcom*cos(qSl-qb)*2.0-rvpp*cos(qSl-qb+qvpp+asin(rcom*sin(qSl-qb)*1.0/sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)))*sqrt(rSl*rSl+rcom*rcom-rSl*rcom*cos(qSl-qb)*2.0)*2.0)+1.0);
    // Error catch
    if ((q > M_PI/2.0) || (q < -M_PI/2.0)) {
        printf("VPP control error: q outside working bounds. q = %f\n", q);
        q  = 0.0;
        dq = 0.0;
    }
    // Tangential toe force to redirect axial leg force to VPP
    ft = fa*tan(q);
    // Derivative of the tangential toe force
    dft = dfa*tan(q) + fa*dq*(pow(tan(q),2.0)+1.0);

    // Add torso control
    forceSl.fx  += -ft*sin(qSl);
    forceSl.dfx += -ft*cos(qSl)*dqSl - dft*sin(qSl);
    forceSl.fz  +=  ft*cos(qSl);
    forceSl.dfz += -ft*sin(qSl)*dqSl + dft*cos(qSl);

    // Use force tracking controller to compute required motor currents
    // Force controller has built-in world coordinate conversion
    std::tie(coSl->motorCurrentA, coSl->motorCurrentB) = ascLegForceSl->control(forceSl, *rsSl, rs.position);
    // Log the forces that it thinks it's applying
    ascLegForceSl->compute(*rsSl, rs.position);

    // Convert leg angle and velocities to torso coordinates
    // PD past q1
    if (qSl < q1) {
        coSl->motorCurrentA += (q1-qSl)*guiIn.leg_pos_kp + (0.0-dqSl)*guiIn.leg_pos_kd;
        coSl->motorCurrentB += (q1-qSl)*guiIn.leg_pos_kp + (0.0-dqSl)*guiIn.leg_pos_kd;
    }
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
    std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.motorAngle, rsSl->halfB.motorAngle);
    std::tie(dqSl, drSl) = ascCommonToolkit.motorVel2LegVel(rsSl->halfA.motorAngle, rsSl->halfB.motorAngle, rsSl->halfA.motorVelocity, rsSl->halfB.motorVelocity);

    // Compute current torso states
    qb = rs.position.bodyPitch;
    dqb = rs.position.bodyPitchVelocity;

    // Convert leg angle and velocities to world coordinates
    qSl  += qb - 3.0*M_PI/2.0;
    dqSl += dqb;

    // Compute gait parameter and velocity
    s = (qSl - q2)/(q3 - q2);
    //ds = dqSl;///(q3 - q2);
    ds = 1.5;

    // Limit gait parameter between zero and one
    //s  = clamp(s, 0.0, 1.0);

    // Make s only increase
    if (s < sPrev) {
        s = sPrev;
        ds = 0.0;
    } else {
        sPrev = s;
    }

    // Use a cubic spline interpolation to slave the flight leg angle to the stance leg angle
    std::tie(qm, dqm) = ascInterpolation.cubic(0.0, 0.90, qeFm, q1, -0.5, 1.0, s, ds);

    // Compute target leg retraction length
    rtFm = r0 - swingLegRetraction;

    // Leg retraction
    //rm = r0 - swingLegRetraction*4.0*(s-pow(s,2.0));
    //drm = -swingLegRetraction*4.0*(1.0-2.0*s);
    // Cubic spline
    if (s < 0.5) {
        // Leg retraction
        std::tie(rm, drm) = ascInterpolation.cubic(0.0, 0.5, reFm, rtFm, -1.0, 0.0, s, ds);
    } else if (s >= 0.5) {
        // Leg extension
        std::tie(rm, drm) = ascInterpolation.cubic(0.5, 0.85, rtFm, r0, 0.0, 0.0, s, ds);
    } else {
        printf("Leg retraction error.  s = %f\n", s);
        rm = rtFm;
        drm = 0.0;
    }
    /*
    // Square wave
    if (s < 0.2) {
        // Leg retraction
        std::tie(rm, drm) = ascInterpolation.cubic(0.0, 0.2, reFm, rtFm, 0.0, 0.0, s, ds);
    } else if ((s >= 0.2) && (s <= 0.65)) {
        rm = rtFm;
        drm = 0.0;
    } else if (s > 0.65) {
        // Leg extension
        std::tie(rm, drm) = ascInterpolation.cubic(0.65, 0.95, rtFm, r0, 0.0, 0.0, s, ds);
    } else {
        printf("Leg retraction error.  s = %f\n", s);
        rm = rtFm;
        drm = 0.0;
    }
    */

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
 *
 * This function computes logical conditionals and uses a decision tree
 * to determine if a single support event has been triggered and responds accordingly.
 */
void ATCSlipWalking::singleSupportEvents(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::robot_state_leg *rsFl, std::deque<double>* filteredToe) {
    // Compute current stance leg states
    std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    std::tie(qFl, rFl) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.legAngle, rsFl->halfB.legAngle);

    // Compute current torso states
    qb = rs.position.bodyPitch;

    // Convert leg angle to world coordinates
    qSl += qb - 3.0*M_PI/2.0;
    qFl += qb - 3.0*M_PI/2.0;

    // Make sure we step forward and don't trigger next state if we back step
    isForwardStep = (qFl <= M_PI/2.0);

    // Handle different trigger methods
    switch (switchMethod) {
        case 0:
            isTrigger = detectStance(rsFl, filteredToe);
            break;

        case 1: // Automatic switch based on gait parameter
            // Stance leg must be ready to take off, and flight leg ready to
            // touch down
            isTrigger = (qSl >= q3) && (qFl <= q2);
            break;
    }

    // Flight leg touch down event (trigger next state)
    if ((isTrigger || isManualSwingLegTD) && isForwardStep) {
        // Advance the walking state machine 1 step and loop to beginning if needed
        walkingState = (walkingState + 1) % 4;
    }
} // singleSupportEvents

/**
 * @brief Double support event triggers.
 * @param rsSl Stance leg robot state pointer.
 * @param rsFl Flight leg robot state pointer.
 * @param ascRateLimitFr0 Flight rest spring length rate limiter pointer.
 * 
 * This function computes logical conditionals and uses a decision tree
 * to determine if a double support event has been triggered and responds accordingly.
 */
void ATCSlipWalking::doubleSupportEvents(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::robot_state_leg *rsFl, ASCRateLimit *ascRateLimitFr0) {
    // Compute current stance leg states
    std::tie(qSl, rSl)   = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    std::tie(dqSl, drSl) = ascCommonToolkit.motorVel2LegVel(rsSl->halfA.legAngle, rsSl->halfB.legAngle, rsSl->halfA.legVelocity, rsSl->halfB.legVelocity);
    std::tie(qFl, rFl)   = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.legAngle, rsFl->halfB.legAngle);
    std::tie(dqFl, drFl) = ascCommonToolkit.motorVel2LegVel(rsFl->halfA.legAngle, rsFl->halfB.legAngle, rsFl->halfA.legVelocity, rsFl->halfB.legVelocity);
    std::tie(qFm, rFm)   = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.motorAngle, rsFl->halfB.motorAngle);

    // Compute current torso states
    qb = rs.position.bodyPitch;
    dqb = rs.position.bodyPitchVelocity;

    // Convert leg angle and velocity to world coordinates
    qSl += qb - 3.0*M_PI/2.0;
    dqSl += dqb;
    qFl += qb - 3.0*M_PI/2.0;
    qFm += qb - 3.0*M_PI/2.0;
    dqFl += dqb;

    // Compute the takeoff leg radial deflection
    rFdefl = rFm - rFl;

    // Handle different trigger methods
    switch (switchMethod) {
        case 0: // When the takeoff ("flight") radial leg deflection is less than ... meters
            isTrigger = (rFdefl <= 0.005);
            break;

        case 1: // Automatic switch based on gait parameter
            isTrigger = (qSl >= q2);
            break;
    }

    // Make sure the flight leg is at a reasonable angle to takeoff
    isForwardStep = (qFl >= M_PI/2.0);

    // Flight leg take off (trigger next state)
    if ((isTrigger || isManualSwingLegTO) && isForwardStep) {
        // Advance the walking state machine 1 step and loop to beginning if needed
        walkingState = (walkingState + 1) % 4;

        // Reset flight control variables
        resetFlightLegParameters(rsFl, ascRateLimitFr0);
    }
} // doubleSupportEvents

void ATCSlipWalking::resetFlightLegParameters(atrias_msgs::robot_state_leg *rsFl, ASCRateLimit *ascRateLimitFr0) {
    // Reset the flight leg rest leg length to neutral
    ascRateLimitFr0->reset(r0);

    // Reset the flight leg progression variable
    s = 0.0;
    sPrev = 0.0;

    // Compute initial flight leg angle and length
    std::tie(qeFm, reFm) = ascCommonToolkit.motorPos2LegPos(rsFl->halfA.motorAngle, rsFl->halfB.motorAngle);
    // Convert to world coordinates
    qeFm += qb - 3.0*M_PI/2.0;
} // resetFlightLegParameters

bool ATCSlipWalking::detectStance(atrias_msgs::robot_state_leg *rsFl, std::deque<double> *filteredToe)
{
    // Touchdown detection
    // Make a baseline by averaging previous values, ignoring the first 20
    double baseline = accumulate(filteredToe->begin()+20.0, filteredToe->end(), 0.0)/(filteredToe->size()-20.0);

    // The threshold for stance is 600 over the baseline reading
    double threshold = 600.0 + baseline;

    // If the toe switch is above the threshold
    if (((double) rsFl->toeSwitch) > threshold) {
        return true;  // The leg is in stance
    }

    // The leg is in flight
    return false;
} // detectStance

void ATCSlipWalking::updateToeFilter(uint16_t newToe, std::deque<double> *filteredToe)
{
    // newToe: New toe measurement
    // filteredToe: A bunch of filtered measurements
    double toe = (double) newToe; // double precision container for newToe

    // Filter to remove bad data
    // If the data is the maximum or minimum the ADC outputs, ignore it
    double prevToe = filteredToe->front();
    if ((newToe == 4095) || (newToe == 0)) {
        toe = prevToe;
    }

    // If the data jumps by more than 1500 and we're not starting up, ignore it
    if ((fabs(prevToe - toe) > 1500.0) && (filteredToe->back() != 5000.0)) {
        toe = prevToe;
    }

    // Rolling average
    // Remove the oldest value
    filteredToe->pop_back();
    // Calculate the average of the 3 most recent values
    int nSamples = 3;
    double average = (accumulate(filteredToe->begin(), filteredToe->begin()+nSamples-1, 0.0) + toe)/((double)nSamples);

    // Store it
    filteredToe->push_front (average);
} // updateToeFilter

ORO_CREATE_COMPONENT(ATCSlipWalking)

} // controller
} // atrias
