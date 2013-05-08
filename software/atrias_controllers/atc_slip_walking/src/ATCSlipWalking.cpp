#include "atc_slip_walking/ATCSlipWalking.hpp"

namespace atrias {
namespace controller {

ATCSlipWalking::ATCSlipWalking(string name) :
    ATC(name),
    commonToolkit(this, "commonToolkit"),
    ascSlipModel(this, "ascSlipModel"),
    ascLegForceLl(this, "ascLegForceLl"),
    ascLegForceRl(this, "ascLegForceRl"),
    hipBoomKinematics(this, "hipBoomKinematics"),
    pdLmA(this, "pdLmA"),
    pdLmB(this, "pdLmB"),
    pdRmA(this, "pdRmA"),
    pdRmB(this, "pdRmB"),
    pdLh(this, "pdLh"),
    pdRh(this, "pdRh"),
    ascRateLimitLmA(this, "ascRateLimitLmA"),
    ascRateLimitLmB(this, "ascRateLimitLmB"),
    ascRateLimitRmA(this, "ascRateLimitRmA"),
    ascRateLimitRmB(this, "ascRateLimitRmB")
{
    // Initially ramp up the current over several seconds
    setStartupEnabled(true);

    // Standing control setup
    legRateLimit = 1.0;

    hipControlSetup();
    eqPointWalkingSetup();
    slipWalkingSetup();
}

void ATCSlipWalking::controller() {
    // Robot state == rs
    // Controller output == co

    if (!isEnabled())
        ascRateLimitLmA.reset(rs.lLeg.halfA.motorAngle);
        ascRateLimitLmB.reset(rs.lLeg.halfB.motorAngle);
        ascRateLimitRmA.reset(rs.rLeg.halfA.motorAngle);
        ascRateLimitRmB.reset(rs.rLeg.halfB.motorAngle);
        return;

    guiCommunication();
    hipControl();

    // Main state machine
    switch (guiIn.main_controller) {
        case 0:
            standingControl();
            break;

        case 1:
            walkingControl();
            break;

        default:
            shutdownControl();
    }

    // Logging
    logOut.dummy = 0.0;
}

void ATCSlipWalking::guiCommunication() {
    // Input angles
    qTO = guiIn.takeoff_angle;
    qTD = guiIn.touchdown_angle;
    rSl = guiIn.stance_leg_length;
    // PD Gains
    pdLmA.P = pdLmB.P = pdRmA.P = pdRmB.P = guiIn.leg_flight_kp;
    pdLmA.D = pdLmB.D = pdRmA.D = pdRmB.D = guiIn.leg_flight_kd;
    pdLh.P = pdRh.P = guiIn.hip_kp;
    pdLh.D = pdRh.D = guiIn.hip_kd;

    // Copy over positions to the GUI output data
    guiOut.isEnabled = isEnabled();
}

void ATCSlipWalking::hipControlSetup() {
    // Circle radii around the boom
    toePosition.left = 2.15;
    toePosition.right = 2.45;
}


void ATCSlipWalking::hipControl() {
    // Hold the hips so the toes walk along a circle
    std::tie(qlh, qrh) = hipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);
    co.lLeg.motorCurrentHip = pdLh(qlh, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentHip = pdRh(qrh, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);
}


void ATCSlipWalking::standingControl() {
    // Leg angles
    qRl = qTD;
    qLl = qTO;
    // Leg lengths
    rRl = rLl = rSl;

    // Compute motor angles
    std::tie(qLmA, qLmB) = commonToolkit.legPos2MotorPos(qLl, rLl);
    std::tie(qRmA, qRmB) = commonToolkit.legPos2MotorPos(qRl, rRl);

    // Rate limit motor velocities
    qLmA = ascRateLimitLmA(qLmA, legRateLimit);
    qLmB = ascRateLimitLmB(qLmB, legRateLimit);
    qRmA = ascRateLimitRmA(qRmA, legRateLimit);
    qRmB = ascRateLimitRmB(qRmB, legRateLimit);

    co.lLeg.motorCurrentA = pdLmA(qLmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
    co.lLeg.motorCurrentB = pdLmB(qLmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
    co.rLeg.motorCurrentA = pdRmA(qRmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
    co.rLeg.motorCurrentB = pdRmB(qRmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
}


void ATCSlipWalking::walkingControl() {
    switch (guiIn.ground_contact_method) {
        case 0: // Manual
            rGC = guiIn.right_ground_contact;
            lGC = guiIn.left_ground_contact;
            break;
        case 1: // Robot configuration
            qRl = (rs.rLeg.halfA.motorAngle+rs.rLeg.halfB.motorAngle)/2.0;
            qLl = (rs.lLeg.halfA.motorAngle+rs.lLeg.halfB.motorAngle)/2.0;
            rGC = (rs.position.zPosition < rSl*sin(qRl));
            lGC = (rs.position.zPosition < rSl*sin(qLl));
        default: // Don't use ground contact
            rGC = true;
            lGC = true;
    }

    switch (guiIn.walking_controller) {
        case 0:
            eqPointWalkingControl(); // To get moving
            break;

        case 1:
            slipWalking();
            break;

        default: // This shouldn't happen
            shutdownControl();
    }
}


void ATCSlipWalking::eqPointWalkingSetup() {
    eqPointState = 0;  // Right leg in stance
    stanceComplete = false;
    flightComplete = false;
    t=0.0;
}

// Control concepts from atc_eq_point
void ATCSlipWalking::eqPointWalkingControl() {
    // Switching conditions
    if ((eqPointState == 1) && (t>0.99) && rGC) {
        eqPointState = 0;  // Right leg in stance
        stanceComplete = false;
        flightComplete = false;
        t=0.0;
    } else if ((eqPointState == 0) && (t>0.99) && lGC) {
        eqPointState = 1;  // Left leg in stance
        stanceComplete = false;
        flightComplete = false;
        t=0.0;
    }

    // Control
    switch (eqPointState) {
        case 0:
            eqPointStanceControl(&rs.rLeg, &co.rLeg, &pdRmA, &pdRmB);
            eqPointFlightControl(&rs.lLeg, &co.lLeg, &pdLmA, &pdLmB);
            break;
        case 1:
            eqPointStanceControl(&rs.lLeg, &co.lLeg, &pdLmA, &pdLmB);
            eqPointFlightControl(&rs.rLeg, &co.rLeg, &pdRmA, &pdRmB);
            break;
        default:
            std::cout << "Unknown eqPointState: " << eqPointState << std::endl;
    }
}

void ATCSlipWalking::eqPointStanceControl(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::controller_output_leg *coSl, ASCPD *pdSmA, ASCPD *pdSmB) {
    // Set gains
    pdSmA->P = pdSmB->P = guiIn.leg_stance_kp;
    pdSmA->D = pdSmB->D = guiIn.leg_stance_kd;
    // Virtual stance leg angle
    qSl = (rsSl->halfA.motorAngle+rsSl->halfB.motorAngle)/2.0;
    // Map stance from touchdown to takeoff (from 0 to 1)
    s = (qSl-qTD) / (qTO-qTD);
    // Desired position for stance motor B
    qSmB_des = qTO + acos(rSl);
    // If the stance leg needs to be rotated
    if ((rsSl->halfB.motorAngle < qSmB_des) && !stanceComplete) {
        // Stance motor A and B angles
        std::tie(qSmA, qSmB) = commonToolkit.legPos2MotorPos(qSl,rSl);
        // Advance position
        coSl->motorCurrentA = pdSmA->operator()(qSmA, rsSl->halfA.motorAngle,0.5,rsSl->halfA.motorVelocity);
        coSl->motorCurrentB = pdSmB->operator()(qSmB, rsSl->halfB.motorAngle,0.5,rsSl->halfB.motorVelocity) + guiIn.stance_current_offset;

    // The stance leg is fully extended
    } else {
        stanceComplete = true;
        // Stance motor A and B angles
        std::tie(qSmA, qSmB) = commonToolkit.legPos2MotorPos(qTO,rSl);
        // Hold position
        coSl->motorCurrentA = pdSmA->operator()(qSmA, rsSl->halfA.motorAngle,0.0,rsSl->halfA.motorVelocity);
        coSl->motorCurrentB = pdSmB->operator()(qSmB, rsSl->halfB.motorAngle,0.0,rsSl->halfB.motorVelocity);
    }
}

void ATCSlipWalking::eqPointFlightControl(atrias_msgs::robot_state_leg *rsFl, atrias_msgs::controller_output_leg *coFl, ASCPD *pdFmA, ASCPD *pdFmB) {
    // Set gains
    pdFmA->P = pdFmB->P = guiIn.leg_flight_kp;
    pdFmA->D = pdFmB->D = guiIn.leg_flight_kd;
    // Virtual flight leg angle
    qFl = (rsFl->halfA.motorAngle+rsFl->halfB.motorAngle)/2.0;
    // Ratchet t along with s
    if (s > t)
        t = s;
    // If the flight leg needs to be rotated
    if ((t < 1.0) && (!flightComplete)) {
        // Amplitude between stance and flight leg lengths
        amp = rSl - guiIn.min_flight_leg_length;
        // Takeoff
        if (t < 0.1) {
            // Length and angle of flight leg
            rFl = rSl - amp * sin(t/guiIn.t_swing*M_PI);
            qFl = qTO;
        // Swing forward
        } else if (t < guiIn.t_swing) {
            rFl = rSl - amp * sin(t/guiIn.t_swing*M_PI);
            qFl = qTO - (qTO-qTD) * (t-0.1) / (guiIn.t_swing-0.1) * (1+guiIn.t_extension);
        // Retract
        } else {
            rFl = rSl;
            qFl = qTD - (qTO-qTD) * (1.0-t) / (1.0-guiIn.t_swing) * guiIn.t_extension;
        }
        // Flight motor A and B angles
        std::tie(qFmA, qFmB) = commonToolkit.legPos2MotorPos(qFl,rFl);
        coFl->motorCurrentA = pdFmA->operator()(qFmA,rsFl->halfA.motorAngle,0.0,rsFl->halfA.motorVelocity);
        coFl->motorCurrentB = pdFmB->operator()(qFmB,rsFl->halfB.motorAngle,0.0,rsFl->halfB.motorVelocity);

    // The other leg finished stance, so this leg is now in stance
    } else {
        flightComplete = true;
        std::tie(qFmA, qFmB) = commonToolkit.legPos2MotorPos(qTD,rSl);
        coFl->motorCurrentA = pdFmA->operator()(qFmA,rsFl->halfA.motorAngle,0.0,rsFl->halfA.motorVelocity);
        coFl->motorCurrentB = pdFmB->operator()(qFmB,rsFl->halfB.motorAngle,0.0,rsFl->halfB.motorVelocity);
    }
}

void ATCSlipWalking::slipWalkingSetup() {
    slipControlState = 0; // Right leg in stance
}

void ATCSlipWalking::slipWalking() {
    // TODO: Add variable comparison that progresses through stance
    if ((slipControlState == 3) && rGC) {
        slipControlState = 0;  // Right leg in stance
    } else if ((slipControlState == 0) && rGC && lGC) {
        slipControlState = 1;  // Both legs in stance
    } else if ((slipControlState == 1) && lGC) {
        slipControlState = 2;  // Left leg in stance
    } else if ((slipControlState == 2) && rGC && lGC) {
        slipControlState = 3;  // Both legs in stance
    }

    // Control
    switch (slipControlState) {
        case 0:
            slipPassiveStanceControl(&rs.rLeg, &co.rLeg, &pdRmA, &pdRmB, &ascLegForceRl);
            slipFlightControl(&rs.lLeg, &co.lLeg, &pdLmA, &pdLmB);
            break;
        case 1:
            slipPassiveStanceControl(&rs.rLeg, &co.rLeg, &pdRmA, &pdRmB, &ascLegForceRl);
            slipPassiveStanceControl(&rs.lLeg, &co.lLeg, &pdLmA, &pdLmB, &ascLegForceLl);
            break;
        case 2:
            slipFlightControl(&rs.rLeg, &co.rLeg, &pdRmA, &pdRmB);
            slipPassiveStanceControl(&rs.lLeg, &co.lLeg, &pdLmA, &pdLmB, &ascLegForceLl);
            break;
        case 3:
            slipPassiveStanceControl(&rs.rLeg, &co.rLeg, &pdRmA, &pdRmB, &ascLegForceRl);
            slipPassiveStanceControl(&rs.lLeg, &co.lLeg, &pdLmA, &pdLmB, &ascLegForceLl);
            break;
        default:
            std::cout << "Unknown slipControlState: " << slipControlState << std::endl;
    }
}

// SLIP force profile control concept from atc_slip_hopping
void ATCSlipWalking::slipActiveStanceControl(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::controller_output_leg *coSl, ASCPD *pdSmA, ASCPD *pdSmB, ASCLegForce *ascLegForceSl) {
    // Spring type
    if (guiIn.spring_type == 0) {
        ascSlipModel.k = commonToolkit.legStiffness(slipState.r, ascSlipModel.r0);
    } else if (guiIn.spring_type == 1) {
        ascSlipModel.k = guiIn.linear_spring_constant;
    }

    // Compute SLIP force profile
    // TODO: Update ascSlipModel for walking
    slipState = ascSlipModel.advanceRK5(slipState);
    legForce = ascSlipModel.force(slipState);

    // TODO: add stance progression variable for flight tracking

    // Set gains
    pdSmA->P = pdSmB->P = guiIn.leg_stance_kp;
    pdSmA->D = pdSmB->D = guiIn.leg_stance_kd;
    ascLegForceSl->kp = guiIn.leg_stance_force_kp;
    ascLegForceSl->kd = guiIn.leg_stance_force_kd;
    ascLegForceSl->ki = 0.0;

    // If the SLIP model says we should be in flight
    // (This should only happen during stance exit)
    if (slipState.isFlight) {
        // Hold the last known stance leg position
        coSl->motorCurrentA = pdSmA->operator()(qSmA, rsSl->halfA.motorAngle, 0.0, rsSl->halfA.motorVelocity);
        coSl->motorCurrentB = pdSmB->operator()(qSmB, rsSl->halfB.motorAngle, 0.0, rsSl->halfB.motorVelocity);
    // Otherwise the SLIP model is in stance
    } else {
        // Store last known leg position
        qSmA = rsSl->halfA.legAngle;
        qSmB = rsSl->halfB.legAngle;

        // Compute and set motor currents
        std::tie(coSl->motorCurrentA, coSl->motorCurrentB) = ascLegForceSl->control(legForce, *rsSl, rs.position);

    }
}

void ATCSlipWalking::slipPassiveStanceControl(atrias_msgs::robot_state_leg *rsSl, atrias_msgs::controller_output_leg *coSl, ASCPD *pdSmA, ASCPD *pdSmB, ASCLegForce *ascLegForceSl) {
    /*
    // Compute leg angle
    std::tie(qSl, rSl) = commonToolkit.motorPos2LegPos(rsSl.halfA.legAngle, rsSl.halfB.legAngle);
    // Set motor angles from the slip model
    std::tie(qSmA, qSmB) = commonToolkit.legPos2MotorPos(ql, ascSlipModel.r0);

    coSl.motorCurrentA = pdSmA->operator()(qSmA, rsSl.halfA.motorAngle, 0.0, rsSl.halfA.motorVelocity);
    coSl.motorCurrentB = pdSmB->operator()(qSmB, rsSl.halfB.motorAngle, 0.0, rsSl.halfB.motorVelocity);
    */
}

void ATCSlipWalking::slipFlightControl(atrias_msgs::robot_state_leg *rsFl, atrias_msgs::controller_output_leg *coFl, ASCPD *pdFmA, ASCPD *pdFmB) {
    // Set gains
    pdFmA->P = pdFmB->P = guiIn.leg_flight_kp;
    pdFmA->D = pdFmB->D = guiIn.leg_flight_kd;
}

void ATCSlipWalking::shutdownControl() {
    // Damping
    co.lLeg.motorCurrentHip = pdLh(0.0, 0.0, 0.0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentHip = pdRh(0.0, 0.0, 0.0, rs.rLeg.hip.legBodyVelocity);
    co.lLeg.motorCurrentA   = pdLmA(0.0, 0.0, 0.0, rs.lLeg.halfA.motorVelocity);
    co.lLeg.motorCurrentB   = pdLmB(0.0, 0.0, 0.0, rs.lLeg.halfB.motorVelocity);
    co.rLeg.motorCurrentA   = pdRmA(0.0, 0.0, 0.0, rs.rLeg.halfA.motorVelocity);
    co.rLeg.motorCurrentB   = pdRmB(0.0, 0.0, 0.0, rs.rLeg.halfB.motorVelocity);
}


// The OROCOS component object
ORO_CREATE_COMPONENT(ATCSlipWalking)

}
}

// vim: expandtab
