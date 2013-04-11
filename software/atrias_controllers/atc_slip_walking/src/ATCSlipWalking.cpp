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
    prevMode = 0;
    // An initial touchdown angle guess
    qTD = 70.0*M_PI/180.0;
    // Circle radii around the boom
    toePosition.left = 2.15;
    toePosition.right = 2.45;
    // Hip gains
    pdHip.P = 100;
    pdHip.D = 2;  // 2 for simulation, 10 for implementation
}

void ATCSlipWalking::controller() {
    // Notation
    // Robot state: rs
    // Controller output: co

    // Do nothing if disabled
    if (!isEnabled())
        return;

    // Setup
    guiCommunication();
    hipControl();

    // Gui State Machine (initial mode is 0)
    if ((guiIn.mode == 0) && (prevMode == 0)) {
        standingControl();
    } else if ((guiIn.mode == 1) && (prevMode != 2)) {
        walkingControl();
        prevMode = 1;
    } else {
        shutdownControl();
        prevMode = 2;
    }

    // Logging
    logOut.dummy = 0.0;
}


void ATCSlipWalking::hipControl() {
    // Hold the hips so the legs walk along a circle
    std::tie(qlh, qrh) = hipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);
    co.lLeg.motorCurrentHip = pdHip(qlh, rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentHip = pdHip(qrh, rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);
}

void ATCSlipWalking::standingControl() {
    //double testVar = commonToolkit.rad2Deg(0.8);

    // Right leg points to qS1
    // Left leg points to qS2
    // SLIP force control

    /*
    co.lLeg.motorCurrentA   = pdLegStance(0, rs.lLeg.halfA.rotorAngle, 0, rs.lLeg.halfA.rotorVelocity);
    co.lLeg.motorCurrentB   = pdLegStance(0, rs.lLeg.halfB.rotorAngle, 0, rs.lLeg.halfB.rotorVelocity);
    co.rLeg.motorCurrentA   = pdLegStance(0, rs.rLeg.halfA.rotorAngle, 0, rs.rLeg.halfA.rotorVelocity);
    co.rLeg.motorCurrentB   = pdLegStance(0, rs.rLeg.halfB.rotorAngle, 0, rs.rLeg.halfB.rotorVelocity);
    */
}

void ATCSlipWalking::walkingControl() {
    // right stance + left swing
    // right swing  + left stance
}

void ATCSlipWalking::shutdownControl() {
    // Damping
    co.lLeg.motorCurrentHip = pdHip(0, 0, 0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentHip = pdHip(0, 0, 0, rs.rLeg.hip.legBodyVelocity);
    co.lLeg.motorCurrentA   = pdLegStance(0, 0, 0, rs.lLeg.halfA.rotorVelocity);
    co.lLeg.motorCurrentB   = pdLegStance(0, 0, 0, rs.lLeg.halfB.rotorVelocity);
    co.rLeg.motorCurrentA   = pdLegStance(0, 0, 0, rs.rLeg.halfA.rotorVelocity);
    co.rLeg.motorCurrentB   = pdLegStance(0, 0, 0, rs.rLeg.halfB.rotorVelocity);
}

void ATCSlipWalking::guiCommunication() {
    // Leg gains
    pdLegStance.P = guiIn.leg_motor_p_gain;
    pdLegStance.D = guiIn.leg_motor_d_gain;
    pdLegFlight.P = guiIn.leg_motor_p_gain;
    pdLegFlight.D = guiIn.leg_motor_d_gain;

    // GUI Output
    guiOut.isEnabled             = isEnabled();
    guiOut.motorPositionLeftA    = rs.lLeg.halfA.rotorAngle;
    guiOut.motorPositionLeftB    = rs.lLeg.halfB.rotorAngle;
    guiOut.motorPositionRightA   = rs.rLeg.halfA.rotorAngle;
    guiOut.motorPositionRightB   = rs.rLeg.halfB.rotorAngle;
    guiOut.motorPositionLeftHip  = rs.lLeg.hip.legBodyAngle;
    guiOut.motorPositionRightHip = rs.rLeg.hip.legBodyAngle;
}

// The OROCOS component object
ORO_CREATE_COMPONENT(ATCSlipWalking)

}
}

// vim: expandtab
