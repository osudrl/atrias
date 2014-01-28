/*! \file ATCMotorSinWave.cpp
 *  \author Andrew Peekema
 *  \brief Component code for the motor sin wave controller
 */

#include <atc_motor_sin_wave/ATCMotorSinWave.hpp>

namespace atrias {
namespace controller {

ATCMotorSinWave::ATCMotorSinWave(string name):
    ATC(name),
    pd0Controller(this, "pd0Controller"),
    pd1Controller(this, "pd1Controller"),
    pd2Controller(this, "pd2Controller"),
    pd3Controller(this, "pd3Controller")
{
    // Initial PD controller gains
    legP = 600;
    pd0Controller.P = legP;
    pd1Controller.P = legP;
    pd2Controller.P = legP;
    pd3Controller.P = legP;

    legD = 15;
    pd0Controller.D = legD;
    pd1Controller.D = legD;
    pd2Controller.D = legD;
    pd3Controller.D = legD;

    // Set Defaults
    targetPos = 0.0;
    currentPos = 0.0;
    targetVel = 0.0;
    currentVel = 0.0;
    t = 0;

    // Slowly increase the current limit to maximum
    setStartupEnabled(true);
}

void ATCMotorSinWave::controller() {
    // If we're disabled
    if (!isEnabled()) {
        // Reset the time counter
        t = 0.0;
    }

    // Make a sinusoidal input
    q = guiIn.leg_ang_amp*sin(t*M_PI*2.0*guiIn.leg_ang_frq);
    dq = 2.0*M_PI*guiIn.leg_ang_amp*cos(t*M_PI*2.0*guiIn.leg_ang_frq);

    // Increment the time counter
    t += 0.001;

    // Set resonable center positions
    centerAAngle = M_PI/4;
    centerBAngle = 3*M_PI/4;
    q += centerAAngle;

    // Set the PD gains
    // Left leg
    // motorA
    pd0Controller.P = guiIn.p_gain;
    pd0Controller.D = guiIn.d_gain;
    // motorB
    pd1Controller.P = guiIn.p_gain;
    pd1Controller.D = guiIn.d_gain;
    // Right leg
    // motorA
    pd2Controller.P = guiIn.p_gain;
    pd2Controller.D = guiIn.d_gain;
    // motorB
    pd3Controller.P = guiIn.p_gain;
    pd3Controller.D = guiIn.d_gain;

    // Left Leg
    // Calculate motorA current
    targetPos = centerAAngle;
    currentPos = rs.lLeg.halfA.motorAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.halfA.motorVelocity;
    co.lLeg.motorCurrentA = pd0Controller(targetPos, currentPos, targetVel, currentVel);

    // Calculate motorB current
    targetPos = centerBAngle;
    currentPos = rs.lLeg.halfB.motorAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.halfB.motorVelocity;
    co.lLeg.motorCurrentB = pd1Controller(targetPos, currentPos, targetVel, currentVel);

    // Right Leg
    // Calculate motorA current
    targetPos = centerAAngle;
    currentPos = rs.rLeg.halfA.motorAngle;
    targetVel = 0.0;
    currentVel = rs.rLeg.halfA.motorVelocity;
    co.rLeg.motorCurrentA = pd2Controller(targetPos, currentPos, targetVel, currentVel);

    // Calculate motorB current
    targetPos = q;
    currentPos = rs.rLeg.halfB.motorAngle;
    targetVel = dq;
    currentVel = rs.rLeg.halfB.motorVelocity;
    co.rLeg.motorCurrentB = pd3Controller(targetPos, currentPos, targetVel, currentVel);

    // Return is enabled
    guiOut.isEnabled = isEnabled();
} // ATCMotorSinWave::()


ORO_CREATE_COMPONENT(ATCMotorSinWave)

} // controller
} // atrias

// vim: expandtab:sts=4
