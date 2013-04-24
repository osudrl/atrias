#include "atc_slip_walking/ATCSlipWalking.hpp"

namespace atrias {
namespace controller {

ATCSlipWalking::ATCSlipWalking(string name) :
    ATC(name),
    commonToolkit(this, "commonToolkit"),
    pdHip(this, "pdHip"),
    pdLegStance(this, "pdLegStance"),
    pdLegFlight(this, "pdLegFlight"),
    hipBoomKinematics(this, "hipBoomKinematics")
{
    // Initially ramp up the current over several seconds
    setStartupEnabled(true);
    // Start standing
    prevState = 0;
    // An initial touchdown angle guess
    qTD = 70.0*M_PI/180.0;
    // Circle radii around the boom
    toePosition.left = 2.15;
    toePosition.right = 2.45;
}

void ATCSlipWalking::controller() {
    // Robot state == rs
    // Controller output == co

    // Do nothing if disabled
    if (!isEnabled())
        return;

    // Setup
    guiCommunication();
    hipControl();

    // Main state machine
    switch (controllerState) {
        case 0:
            standingControl();
            break;

        case 1:
            walkingControl();
            break;

        default:
            shutdownControl();
            break;
    }

    // Logging
    logOut.dummy = 0.0;
}

void ATCSlipWalking::guiCommunication() {
    // GUI values
    controllerState = guiIn.main_controller;

    // PD Gains
    pdLegStance.P = guiIn.leg_stance_kp;
    pdLegStance.D = guiIn.leg_stance_kd;
    pdLegFlight.P = guiIn.leg_flight_kp;
    pdLegFlight.D = guiIn.leg_flight_kd;
    pdHip.P       = guiIn.hip_kp;
    pdHip.D       = guiIn.hip_kd;

    // Copy over positions to the GUI output data
    guiOut.isEnabled = isEnabled();
}


void ATCSlipWalking::hipControl() {
    // Hold the hips so the legs walk along a circle
    std::tie(qlh, qrh) = hipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);
    co.lLeg.motorCurrentHip = pdHip(qlh, rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentHip = pdHip(qrh, rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);
}


void ATCSlipWalking::standingControl() {
    // Leg angles
    qrl = guiIn.touchdown_angle;
    qll = guiIn.takeoff_angle;
    // Leg lengths
    rrl = rll = guiIn.standing_leg_length;

    // Compute motor angles
    std::tie(qlmA, qlmB) = commonToolkit.legPos2MotorPos(qll, rrl);
    std::tie(qrmA, qrmB) = commonToolkit.legPos2MotorPos(qll, rrl);

    co.lLeg.motorCurrentA = pdLegStance(qlmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
    co.lLeg.motorCurrentB = pdLegStance(qlmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
    co.rLeg.motorCurrentA = pdLegStance(qrmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
    co.rLeg.motorCurrentB = pdLegStance(qrmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
}


void ATCSlipWalking::walkingControl() {
    // Uses the control concept from atc_eq_point
    // right stance + left swing
    // right swing  + left stance

    // Uses the control concept from atc_slip_hopping

    /*
    // Gui State Machine (initial mode is 0)
    if ((guiIn.mode == 0) && (prevState == 0)) {
        standingControl();
    } else if ((guiIn.mode == 1) && (prevState != 2)) {
        walkingControl();
        prevState = 1;
    } else {
        shutdownControl();
        prevState = 2;
    }
    */
}

void ATCSlipWalking::shutdownControl() {
    // Damping
    co.lLeg.motorCurrentHip = pdHip(0, 0, 0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentHip = pdHip(0, 0, 0, rs.rLeg.hip.legBodyVelocity);
    co.lLeg.motorCurrentA   = pdLegStance(0, 0, 0, rs.lLeg.halfA.motorVelocity);
    co.lLeg.motorCurrentB   = pdLegStance(0, 0, 0, rs.lLeg.halfB.motorVelocity);
    co.rLeg.motorCurrentA   = pdLegStance(0, 0, 0, rs.rLeg.halfA.motorVelocity);
    co.rLeg.motorCurrentB   = pdLegStance(0, 0, 0, rs.rLeg.halfB.motorVelocity);
}


// The OROCOS component object
ORO_CREATE_COMPONENT(ATCSlipWalking)

}
}

// vim: expandtab
