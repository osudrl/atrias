#include "atc_slip_hopping/ATCSlipHopping.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

ATCSlipHopping::ATCSlipHopping(string name) :
    ATC(name),
    ascCommonToolkit(this, "ascCommonToolkit"),
    ascHipBoomKinematics(this, "ascHipBoomKinematics"),
    ascInterpolation(this, "ascInterpolation"),
    ascSlipModel(this, "ascSlipModel"),
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
    legRateLimit = 0.5;
    hipRateLimit = 0.5;

    // Initialize hopping state
    hoppingState = 0;
}


/**
 * @brief This is the main function for the top-level controller.
 * The ATC class automatically handles startup and shutdown,
 * if they are not disabled.
 */
void ATCSlipHopping::controller() {
    // Update misc. controller parameters and sub controllers
    updateController();

    // Update GUI
    updateGui();

    // Run hip controller
    hipController();

    // Main controller state machine
    switch (controllerState) {
        case 0: // Standing controller
            standingController();

            // Reset hopping state machine
            hoppingState = 0;
            break;

        case 1: // SLIP hopping controller
            // SLIP hopping controller state machine
            switch (hoppingState) {
                case 0: // Flight phase
                    ballisticStanceLegController(&rs.lLeg, &co.lLeg, &ascPDLmA, &ascPDLmB, &ascRateLimitLmA, &ascRateLimitLmB);
                    ballisticStanceLegController(&rs.rLeg, &co.rLeg, &ascPDRmA, &ascPDRmB, &ascRateLimitRmA, &ascRateLimitRmB);
                    ballisticEvents();
                    break;

                case 1: // Stance phase
                    switch (stanceControlType) {
                        case 0:
                            passiveStanceController(&rs.lLeg, &co.lLeg, &ascPDLmA, &ascPDLmB);
                            passiveStanceController(&rs.rLeg, &co.rLeg, &ascPDRmA, &ascPDRmB);
                            break;

                        case 1:
                            //updateSlipForces();
                            //slipForceStanceController(&rs.lLeg, &co.lLeg, &ascPDLmA, &ascPDLmB, &ascLegForceL);
                            //slipForceStanceController(&rs.rLeg, &co.rLeg, &ascPDRmA, &ascPDRmB, &ascLegForceR);
                            break;

                        case 2:
                            virtualSpringStanceController(&rs.lLeg, &co.lLeg, &ascLegForceL);
                            virtualSpringStanceController(&rs.rLeg, &co.rLeg, &ascLegForceR);
                            break;
                    }
                    stanceEvents();
                    break;
            }
            break;

        case 2: // Shutdown
            shutdownController();
            break;
    }
}


/**
 * @brief This function handles all of the non-controller related updating.
 */
void ATCSlipHopping::updateController() {
    // If we are disabled or the controller combobox has been switched since the
    // last cycle, reset rate limiters to prevent jumps in motor positions
    if (!(isEnabled()) || !(controllerState == guiIn.main_controller)) {
        ascRateLimitLmA.reset(rs.lLeg.halfA.motorAngle);
        ascRateLimitLmB.reset(rs.lLeg.halfB.motorAngle);
        ascRateLimitRmA.reset(rs.rLeg.halfA.motorAngle);
        ascRateLimitRmB.reset(rs.rLeg.halfB.motorAngle);
        ascRateLimitLh.reset(rs.lLeg.hip.legBodyAngle);
        ascRateLimitRh.reset(rs.rLeg.hip.legBodyAngle);
    }

    // Compute the leg component forces from current spring deflection and
    // leg configuration.
    ascLegForceL.compute(rs.lLeg, rs.position);
    ascLegForceR.compute(rs.rLeg, rs.position);
}


/**
 * @brief This function handles all communication to and from the GUI.
 */
void ATCSlipHopping::updateGui() {
    // Get controller options
    controllerState = guiIn.main_controller;
    hoppingType = guiIn.hop_type;
    stanceControlType = guiIn.stance_controller;
    forceControlType = guiIn.force_type;
    springType = guiIn.spring_type;

    // Event and state trigger parameters
    forceThresholdTO = guiIn.force_threshold_td;
    forceThresholdTO = guiIn.force_threshold_to;
    positionThresholdTD = guiIn.position_threshold_td;

    // Get SLIP model parameters
    //ascSlipModel.r0 = guiIn.slip_leg;
    r0 = guiIn.slip_leg;
    hopHeight = guiIn.hop_height;

    // Get leg motor PD position controller gains
    ascPDLmA.P = ascPDLmB.P = ascPDRmA.P = ascPDRmB.P = guiIn.leg_pos_kp;
    ascPDLmA.D = ascPDLmB.D = ascPDRmA.D = ascPDRmB.D = guiIn.leg_pos_kd;

    // Get leg motor PD force controller gains
    ascLegForceL.kp = ascLegForceR.kp = guiIn.leg_for_kp;
    ascLegForceL.ki = ascLegForceR.ki = 0.0;
    ascLegForceL.kd = ascLegForceR.kd = guiIn.leg_for_kd;

    // Get hip motor PD position controller gains
    ascPDLh.P = ascPDRh.P = guiIn.hip_pos_kp;
    ascPDLh.D = ascPDRh.D = guiIn.hip_pos_kd;

    // Set controller options
    guiOut.isEnabled = isEnabled();
    guiOut.hopping_state = hoppingState;
}


/**
 * @brief This function handles all hip motor commands independent of other
 * functions. It works by computing the inverse kinematics of the robot
 * selecting hip angles that result in the desired toe positions. This keeps
 * knee torques to a minimum.
 */
void ATCSlipHopping::hipController() {
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
void ATCSlipHopping::standingController() {
    // Set leg configuration
    qLl = qRl = M_PI/2.0; // Leg angles
    rLl = rRl = guiIn.standing_leg; // Leg length

    // Compute motor angles from desired leg configuration
    std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(qLl, rLl);
    std::tie(qRmA, qRmB) = ascCommonToolkit.legPos2MotorPos(qRl, rRl);

    // Rate limit motor velocities to smooth step inputs
    qLmA = ascRateLimitLmA(qLmA, legRateLimit);
    qLmB = ascRateLimitLmB(qLmB, legRateLimit);
    qRmA = ascRateLimitRmA(qRmA, legRateLimit);
    qRmB = ascRateLimitRmB(qRmB, legRateLimit);

    // Compute and set motor currents from position based PD controllers
    co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
    co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
    co.rLeg.motorCurrentA = ascPDRmA(qRmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
    co.rLeg.motorCurrentB = ascPDRmB(qRmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
}


/**
 * @brief This function imposes virtual dampers on each motor allowing the
 * robot to safely and slowly shutdown.
 */
void ATCSlipHopping::shutdownController() {
    // Compute and set motor currents from position based PD controllers
    co.lLeg.motorCurrentA = ascPDLmA(0.0, 0.0, 0.0, rs.lLeg.halfA.motorVelocity);
    co.lLeg.motorCurrentB = ascPDLmB(0.0, 0.0, 0.0, rs.lLeg.halfB.motorVelocity);
    co.lLeg.motorCurrentHip = ascPDLh(0.0, 0.0, 0.0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentA = ascPDRmA(0.0, 0.0, 0.0, rs.rLeg.halfA.motorVelocity);
    co.rLeg.motorCurrentB = ascPDRmB(0.0, 0.0, 0.0, rs.rLeg.halfB.motorVelocity);
    co.rLeg.motorCurrentHip = ascPDRh(0.0, 0.0, 0.0, rs.rLeg.hip.legBodyVelocity);
}


/**
 * @brief This function computes the current virtual spring stiffness of a leg
 * and advances the SLIP model integrator.
 */
/*void ATCSlipHopping::updateSlipForces() {
    // Spring type and stiffness
    if (springType == 0) {
        // Compute current ATRIAS non-linear spring constant for given leg configuration
        std::tie(ascSlipModel.k, ascSlipModel.dk) = ascCommonToolkit.legStiffness(slipState.r, slipState.dr, ascSlipModel.r0);

    } else if (springType == 1) {
        // Desired linear stiffness
        ascSlipModel.k = guiIn.slip_spring;
        ascSlipModel.dk = 0.0;
    }

    // Set SLIP model parameters, double stiffness if two leg hopping
    if (hoppingType == 2) {
        ascSlipModel.k = 2.0*ascSlipModel.k;
        ascSlipModel.dk = 2.0*ascSlipModel.dk;
    }

    //printf("k dk %f %f\n", ascSlipModel.k, ascSlipModel.dk);
    //printf("before r q dr dq  %f %f %f %f\n", slipState.r, slipState.q, slipState.dr, slipState.dq);

    // Compute SLIP force profile
    slipState = ascSlipModel.advanceRK4(slipState);
    forceS = ascSlipModel.force(slipState);

    // Halve the force if two leg hopping
    if (hoppingType == 2) {
        forceS.fx = forceS.fx/2.0;
        forceS.fz = forceS.fz/2.0;
        forceS.dfx = forceS.dfx/2.0;
        forceS.dfz = forceS.dfz/2.0;
    }

    //printf("after: %f %f %f %f\n", slipState.r, slipState.q, slipState.dr, slipState.dq);
    //printf("forces: %f %f %f %f\m", forceS.fx, forceS.fz, forceS.dfx, forceS.dfz);
}*/


/**
 * @brief A SLIP based force tracking stance phase controller. FIXME
 */
/*void ATCSlipHopping::slipForceStanceController(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::controller_output_leg *coSl, ASCPD *ascPDSmA, ASCPD *ascPDSmB, ASCLegForce *ascLegForceS) {
    // Check if SLIP model says we should be in flight or stance
    if (slipState.isFlight) {
        // Use last know leg position from stance
        coSl->motorCurrentA = ascPDSmA->operator()(qSmA, rsSl->halfA.motorAngle, 0.0, rsSl->halfA.motorVelocity);
        coSl->motorCurrentB = ascPDSmB->operator()(qSmB, rsSl->halfB.motorAngle, 0.0, rsSl->halfB.motorVelocity);

    } else {
        // Store last known leg position // FIXME: conflict when two leg hopping because this is called twice
        qSmA = rsSl->halfA.legAngle;
        qSmB = rsSl->halfB.legAngle;

        // Compute and set motor currents from force based PID controllers
        std::tie(coSl->motorCurrentA, coSl->motorCurrentB) = ascLegForceS->control(forceS, *rsSl, rs.position);
    }
}*/


/**
 * @brief A simple stance phase controller allowing only leg length
 * forces with zero leg angle torques. Uses a position controller to
 * keep virtual motor leg length constant while minimizing spring
 * about the hip. Can be used with mechanical motor lock device.
 */
void ATCSlipHopping::passiveStanceController(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::controller_output_leg *coSl, ASCPD *ascPDSmA, ASCPD *ascPDSmB) {
    // Compute current leg angle and length
    std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);

    // Compute and set motor angles such that there is no hip torque, only
    // axial leg forces
    std::tie(qSmA, qSmB) = ascCommonToolkit.legPos2MotorPos(qSl, ascSlipModel.r0);

    // Compute and set motor currents from position based PD controllers
    coSl->motorCurrentA = ascPDSmA->operator()(qSmA, rsSl->halfA.motorAngle, 0.0, rsSl->halfA.motorVelocity);
    coSl->motorCurrentB = ascPDSmB->operator()(qSmB, rsSl->halfB.motorAngle, 0.0, rsSl->halfB.motorVelocity);
}


/**
 * @brief A simple stance phase controller simulating a virtual spring
 * between the hip and toe. Uses a force controller to then track these
 * forces that are based on leg deflection.
 */
void ATCSlipHopping::virtualSpringStanceController(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::controller_output_leg *coSl, ASCLegForce *ascLegForceS) {
    // Compute current leg angle and length
    std::tie(qSl, rSl) = ascCommonToolkit.motorPos2LegPos(rsSl->halfA.legAngle, rsSl->halfB.legAngle);
    std::tie(dqSl, drSl) = ascCommonToolkit.motorVel2LegVel(rsSl->halfA.legAngle, rsSl->halfB.legAngle, rsSl->halfA.legVelocity, rsSl->halfB.legVelocity);

    // Compute current ATRIAS non-linear spring constant for given leg configuration
    std::tie(fa, dfa) = ascCommonToolkit.legForce(rSl, drSl, r0);

    // Define component forces and their derivatives
    forceS.fx = -fa*cos(qSl);
    forceS.dfx = dqSl*sin(qSl)*fa - dfa*cos(qSl);
    forceS.fz = fa*sin(qSl);
    forceS.dfz = dqSl*cos(qSl)*fa + dfa*sin(qSl);

    // Use force tracking controller to compute required motor currents
    std::tie(coSl->motorCurrentA, coSl->motorCurrentB) = ascLegForceS->control(forceS, *rsSl, rs.position);
}


/**
 * @brief A simple constant leg position flight phase controller.
 */
void ATCSlipHopping::ballisticFlightLegController(atrias_msgs::robot_state_leg *rsFl, atrias_msgs::controller_output_leg *coFl, ASCPD *ascPDFmA, ASCPD *ascPDFmB, ASCRateLimit *ascRateLimitFmA, ASCRateLimit *ascRateLimitFmB) {
    // Compute motor angles from desired leg configuration
    std::tie(qFmA, qFmB) = ascCommonToolkit.legPos2MotorPos(M_PI/2.0, ascSlipModel.r0*0.85);

    // Rate limit motor velocities to smooth step inputs
    qFmA = ascRateLimitFmA->operator()(qFmA, legRateLimit);
    qFmB = ascRateLimitFmB->operator()(qFmB, legRateLimit);

    // Compute and set motor currents from position based PID controllers
    coFl->motorCurrentA = ascPDFmA->operator()(qFmA, rsFl->halfA.motorAngle, 0.0, rsFl->halfA.motorVelocity);
    coFl->motorCurrentB = ascPDFmB->operator()(qFmB, rsFl->halfB.motorAngle, 0.0, rsFl->halfB.motorVelocity);
}


/**
 * @brief A simple constant leg position stance phase controller.
 */
void ATCSlipHopping::ballisticStanceLegController(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::controller_output_leg *coSl, ASCPD *ascPDSmA, ASCPD *ascPDSmB, ASCRateLimit *ascRateLimitSmA, ASCRateLimit *ascRateLimitSmB) {
    // Compute motor angles from desired leg configuration
    std::tie(qSmA, qSmB) = ascCommonToolkit.legPos2MotorPos(M_PI/2.0, ascSlipModel.r0);

    // Rate limit motor velocities to smooth step inputs
    qSmA = ascRateLimitSmA->operator()(qSmA, legRateLimit);
    qSmB = ascRateLimitSmB->operator()(qSmB, legRateLimit);

    // Compute and set motor currents from position based PID controllers
    coSl->motorCurrentA = ascPDSmA->operator()(qSmA, rsSl->halfA.motorAngle, 0.0, rsSl->halfA.motorVelocity);
    coSl->motorCurrentB = ascPDSmB->operator()(qSmB, rsSl->halfB.motorAngle, 0.0, rsSl->halfB.motorVelocity);
}


/**
 * @brief Updates the intial conditions for the SLIP model integrator.
 */
/*void ATCSlipHopping::updateSlipConditions() {
    // Redefine slip initial conditions incase we go into stance next time step
    switch (forceControlType) {
        case 0: // Updated initial conditions (apex tracking)
            slipState.r = ascSlipModel.r0;
            slipState.q = M_PI/2.0;
            std::tie(slipState.dq, slipState.dr) = ascCommonToolkit.cartVel2PolVel(slipState.q, slipState.r, rs.position.xVelocity, rs.position.zVelocity);
            break;

        case 1: // Non-updated intitial conditions (terrain following)
            slipState.r = ascSlipModel.r0;
            slipState.q = M_PI/2.0;
            slipState.dq = 0.0;
            slipState.dr = -sqrt(2.0*9.81*hopHeight);
            break;
    }
}*/


/**
 * @brief This function computes logical conditionals and uses a decision tree
 * to determine if a stance event has been triggered and responds accordingly.
 */
void ATCSlipHopping::stanceEvents() {
    // Compute the current leg angles and lengths
    std::tie(qLl, rLl) = ascCommonToolkit.motorPos2LegPos(rs.lLeg.halfA.legAngle, rs.lLeg.halfB.legAngle);
    std::tie(qRl, rRl) = ascCommonToolkit.motorPos2LegPos(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle);

    // Compute the leg component forces from current spring deflection and
    // leg configuration
    forceL = ascLegForceL.compute(rs.lLeg, rs.position);
    forceR = ascLegForceR.compute(rs.rLeg, rs.position);

    // Compute logical conditionals for event triggers
    isLeftLegTO = (forceL.fz >= -forceThresholdTO);
    isRightLegTO = (forceR.fz >= -forceThresholdTO);

    // Single support event trigger decision tree
    if (isLeftLegTO && isRightLegTO) {
        // Advance the state machine 1 step or loop to beginning
        hoppingState = (hoppingState + 1) % 2;
    }
}


/**
 * @brief This function computes logical conditionals and uses a decision tree
 * to determine if a ballistic event has been triggered and responds
 * accordingly.
 */
void ATCSlipHopping::ballisticEvents() {
    // Compute the current leg angles and lengths
    std::tie(qLl, rLl) = ascCommonToolkit.motorPos2LegPos(rs.lLeg.halfA.legAngle, rs.lLeg.halfB.legAngle);
    std::tie(qRl, rRl) = ascCommonToolkit.motorPos2LegPos(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle);

    // Compute the leg component forces from current spring deflection and
    // leg configuration.
    forceL = ascLegForceL.compute(rs.lLeg, rs.position);
    forceR = ascLegForceR.compute(rs.rLeg, rs.position);

    // Compute logical conditionals for event triggers
    isLeftLegTD = (forceL.fz <= -forceThresholdTD) && (rLl*sin(qLl) >= rs.position.zPosition - positionThresholdTD);
    isRightLegTD = (forceL.fz <= -forceThresholdTD) && (rLl*sin(qLl) >= rs.position.zPosition - positionThresholdTD);

    // Ballistic event trigger decision tree
    if (isLeftLegTD || isRightLegTD) {
        // Advance the walking state machine 1 step or loop to beginning
        hoppingState = (hoppingState + 1) % 2;

        // Update SLIP model initial conditions
        //updateSlipConditions();
    }
}


// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCSlipHopping)

}
}
