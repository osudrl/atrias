#include "atc_slip_walking/ATCSlipWalking.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// This constructor call is much simpler.
ATCSlipWalking::ATCSlipWalking(string name) :
    ATC(name),
    pdLA(this, "pdLA"),
    pdLB(this, "pdLB"),
    pdLH(this, "pdLHip"),
    pdRA(this, "pdRA"),
    pdRB(this, "pdRB"),
    pdRH(this, "pdRHip"),
    rateLimLA(this, "rateLimLA"),
    rateLimLB(this, "rateLimLB"),
    rateLimLH(this, "rateLimLHip"),
    rateLimRA(this, "rateLimRA"),
    rateLimRB(this, "rateLimRB"),
    rateLimRH(this, "rateLimRHip")
{
    // We don't need to do much here, just call the ATC() constructor above.
}

void ATCSlipWalking::controller() {
    // The robot state is in an inherited member variable named rs
    // The controller output should be put in the inherited member co

    // The maximum rate of motion
    double rate = 1.0;

    // Reset the rate limiters if we're disabled. If not, disabling,
    // moving the robot or the setpoints, then enabling produces
    // a jump.
    if (!isEnabled()) {
        rateLimLA.reset(rs.lLeg.halfA.rotorAngle);
        rateLimLB.reset(rs.lLeg.halfB.rotorAngle);
        rateLimLH.reset(rs.lLeg.hip.legBodyAngle);
        rateLimRA.reset(rs.rLeg.halfA.rotorAngle);
        rateLimRB.reset(rs.rLeg.halfB.rotorAngle);
        rateLimRH.reset(rs.rLeg.hip.legBodyAngle);
    }

    /* The logOut class is an inherited member from ATC, and is of type
     * controller_log_data. guiIn, similarly, is of type
     * gui_to_controller
     */
    logOut.lATgt = rateLimLA(guiIn.des_motor_ang_left_A,    rate);
    logOut.lBTgt = rateLimLB(guiIn.des_motor_ang_left_B,    rate);
    logOut.lHTgt = rateLimLH(guiIn.des_motor_ang_left_hip,  rate);
    logOut.rATgt = rateLimRA(guiIn.des_motor_ang_right_A,   rate);
    logOut.rBTgt = rateLimRB(guiIn.des_motor_ang_right_B,   rate);
    logOut.rHTgt = rateLimRH(guiIn.des_motor_ang_right_hip, rate);

    // Set gains
    // Legs
    pdLA.P = pdLB.P = pdRA.P = pdRB.P = guiIn.leg_motor_p_gain;
    pdLA.D = pdLB.D = pdRA.D = pdRB.D = guiIn.leg_motor_d_gain;
    // Hips
    pdLH.P = pdRH.P = guiIn.hip_motor_p_gain;
    pdLH.D = pdRH.D = guiIn.hip_motor_d_gain;

    // Command the outputs (and copy to our logging data).
    // This is where the definition of ASCPD as a functor is convenient.
    // Legs
    co.lLeg.motorCurrentA = pdLA(logOut.lATgt, rs.lLeg.halfA.rotorAngle, 0, rs.lLeg.halfA.rotorVelocity);
    co.lLeg.motorCurrentB = pdLB(logOut.lBTgt, rs.lLeg.halfB.rotorAngle, 0, rs.lLeg.halfB.rotorVelocity);
    co.rLeg.motorCurrentA = pdRA(logOut.rATgt, rs.rLeg.halfA.rotorAngle, 0, rs.rLeg.halfA.rotorVelocity);
    co.rLeg.motorCurrentB = pdRB(logOut.rBTgt, rs.rLeg.halfB.rotorAngle, 0, rs.rLeg.halfB.rotorVelocity);
    // Hips
    co.lLeg.motorCurrentHip = pdLH(logOut.lHTgt, rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
    co.rLeg.motorCurrentHip = pdRH(logOut.rHTgt, rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);

    /* Copy over positions to the GUI output data
     * Suppose that the guiOut message has the following members:
     *     float64 posA
     *     float64 posB
     */
    guiOut.isEnabled             = isEnabled();
    guiOut.motorPositionLeftA    = rs.lLeg.halfA.rotorAngle;
    guiOut.motorPositionLeftB    = rs.lLeg.halfB.rotorAngle;
    guiOut.motorPositionRightA   = rs.rLeg.halfA.rotorAngle;
    guiOut.motorPositionRightB   = rs.rLeg.halfB.rotorAngle;
    guiOut.motorPositionLeftHip  = rs.lLeg.hip.legBodyAngle;
    guiOut.motorPositionRightHip = rs.rLeg.hip.legBodyAngle;

    /* Additionally, the following functions are available to command the robot state:
     * commandHalt();    // Trigger a Halt
     * commandEStop();   // Trigger an EStop
     * commandDisable(); // Disable the robot
     * setStartupEnabled(true/false) // Enable or disable the default startup controller
     * setShutdownEnabled(true/false) // Enable or disable the shutdown controller
     */
}

// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCSlipWalking)

}
}

// vim: expandtab
