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
    // Initial mode is 0
    prevMode = 0;
    // An initial touchdown angle guess
    qTD = 70.0*M_PI/180.0;
    // Circle radii around the boom
    toePosition.left = 2.15;
    toePosition.right = 2.45;
    // Hip gains
    pdHip.P = 100;
    pdHip.D = 2;//10;
}

void ATCSlipWalking::controller() {
    // Notation
    // Robot state: rs
    // Controller output: co

    //double testVar = commonToolkit.rad2Deg(0.8);

    // Do nothing if disabled
    if (!isEnabled())
        return;

    // Leg gains
    pdLegStance.P = guiIn.leg_motor_p_gain;
    pdLegStance.D = guiIn.leg_motor_d_gain;
    pdLegFlight.P = guiIn.leg_motor_p_gain;
    pdLegFlight.D = guiIn.leg_motor_d_gain;

    // Hold the hips so the legs walk along a circle
    std::tie(qlh, qrh) = hipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);
    co.lLeg.motorCurrentHip = pdHip(qlh, rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentHip = pdHip(qrh, rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);

    // Initialize legs (gui radio button, default)
    /*
    if ((guiIn.mode == 0) && (prevMode == 0))
    {
        // Right leg points to qS1
        // Left leg points to qS2
        // SLIP force control

        // The current mode
        prevMode = 0;
    }
    // Walk (gui radio button)
    else if ((guiIn.mode == 1) && (prevMode != 2))
    {
        // right stance + left swing
        // right swing  + left stance

        // The current mode
        prevMode = 1;
    }
    // Stop walking (gui radio button)
    else if (guiIn.mode == 2)
    {
        // Damping hip control
        co.lLeg.motorCurrentHip = pdHip(0, 0, 0, rs.lLeg.hip.legBodyVelocity);
        co.rLeg.motorCurrentHip = pdHip(0, 0, 0, rs.rLeg.hip.legBodyVelocity);

        // The current mode
        prevMode = 2;
    }
    */


    // Controller output
    /*
    co.lLeg.motorCurrentA   = pdLegStance(0, rs.lLeg.halfA.rotorAngle, 0, rs.lLeg.halfA.rotorVelocity);
    co.lLeg.motorCurrentB   = pdLegStance(0, rs.lLeg.halfB.rotorAngle, 0, rs.lLeg.halfB.rotorVelocity);
    co.rLeg.motorCurrentA   = pdLegStance(0, rs.rLeg.halfA.rotorAngle, 0, rs.rLeg.halfA.rotorVelocity);
    co.rLeg.motorCurrentB   = pdLegStance(0, rs.rLeg.halfB.rotorAngle, 0, rs.rLeg.halfB.rotorVelocity);
    */

    // GUI Output
    guiOut.isEnabled             = isEnabled();
    guiOut.motorPositionLeftA    = rs.lLeg.halfA.rotorAngle;
    guiOut.motorPositionLeftB    = rs.lLeg.halfB.rotorAngle;
    guiOut.motorPositionRightA   = rs.rLeg.halfA.rotorAngle;
    guiOut.motorPositionRightB   = rs.rLeg.halfB.rotorAngle;
    guiOut.motorPositionLeftHip  = rs.lLeg.hip.legBodyAngle;
    guiOut.motorPositionRightHip = rs.rLeg.hip.legBodyAngle;

    // Logging
    logOut.dummy = 0.0;
}

// The OROCOS component object
ORO_CREATE_COMPONENT(ATCSlipWalking)

}
}

// vim: expandtab
