#include "atc_slip_walking/ATCSlipWalking.hpp"

namespace atrias {
namespace controller {

ATCSlipWalking::ATCSlipWalking(string name) :
    ATC(name),
    pdLeg(this, "pdLeg"),
    pdHip(this, "pdHip")
{
    // Nothing is needed
}

void ATCSlipWalking::controller() {
    // Notation
    // Robot state: rs
    // Controller output: co

    // Do nothing if disabled
    if (!isEnabled())
        return;

    // Startup is handled by the ATC class.  It slowly ramps up
    // over several seconds.  Set to false to disable
    // setStartupEnabled(true)

    // Set gains
    // Legs
    pdLeg.P = guiIn.leg_motor_p_gain;
    pdLeg.D = guiIn.leg_motor_d_gain;
    // Hips
    pdHip.P = guiIn.hip_motor_p_gain;
    pdHip.D = guiIn.hip_motor_d_gain;

    // Command the outputs (and copy to our logging data).
    // Legs
    co.lLeg.motorCurrentA = pdLeg(0, rs.lLeg.halfA.rotorAngle, 0, rs.lLeg.halfA.rotorVelocity);
    co.lLeg.motorCurrentB = pdLeg(0, rs.lLeg.halfB.rotorAngle, 0, rs.lLeg.halfB.rotorVelocity);
    co.rLeg.motorCurrentA = pdLeg(0, rs.rLeg.halfA.rotorAngle, 0, rs.rLeg.halfA.rotorVelocity);
    co.rLeg.motorCurrentB = pdLeg(0, rs.rLeg.halfB.rotorAngle, 0, rs.rLeg.halfB.rotorVelocity);
    // Hips
    co.lLeg.motorCurrentHip = pdHip(0, rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentHip = pdHip(0, rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);

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
