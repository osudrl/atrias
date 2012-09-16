/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the leg sin wave controller
 */

#include <atc_leg_sin_wave/controller_component.h>

namespace atrias {
namespace controller {

ATCLegSinWave::ATCLegSinWave(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCLegSinWave::runController, this, OwnThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add properties.
    this->addProperty("pd0Name", pd0Name)
        .doc("Name of 0th PD subcontroller.");
    this->addProperty("pd1Name", pd1Name)
        .doc("Name of 1st PD subcontroller.");
    this->addProperty("pd2Name", pd2Name)
        .doc("Name of 2th PD subcontroller.");
    this->addProperty("pd3Name", pd3Name)
        .doc("Name of 3th PD subcontroller.");
    this->addProperty("sin0Name", sin0Name)
        .doc("Name of 0th sin generator subcontroller.");
    this->addProperty("sin1Name", sin1Name)
        .doc("Name of 1st sin generator subcontroller.");
    this->addProperty("sin2Name", sin2Name)
        .doc("Name of 2th sin generator subcontroller.");
    this->addProperty("sin3Name", sin3Name)
        .doc("Name of 3th sin generator subcontroller.");

    // Add ports.
    addEventPort(guiDataIn);

    // Set Defaults
    targetPos = 0;
    currentPos = 0;
    targetVel = 0;
    currentVel = 0;

    log(Info) << "[ATCLSW] Leg sin wave controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCLegSinWave::runController(atrias_msgs::robot_state rs) {
    // Only run the controller when we're enabled
    if ((uint8_t)rs.cmState != (uint8_t)controllerManager::RtOpsCommand::ENABLE)
    {
        // Do nothing
        co.lLeg.motorCurrentA = 0.0;
        co.lLeg.motorCurrentB = 0.0;
        co.lLeg.motorCurrentHip = 0.0;
        co.rLeg.motorCurrentA = 0.0;
        co.rLeg.motorCurrentB = 0.0;
        co.rLeg.motorCurrentHip = 0.0;
        return co;
    }

    // Get a sinusoidal input
    lLegLen = sin0Controller(guiIn.leg_len_frq, guiIn.leg_len_amp);
    lLegAng = sin1Controller(guiIn.leg_ang_frq, guiIn.leg_ang_amp);
    rLegLen = sin2Controller(guiIn.leg_len_frq, guiIn.leg_len_amp);
    rLegAng = sin3Controller(guiIn.leg_ang_frq, guiIn.leg_ang_amp);
    // lLegLen.ang / .vel

    // Set resonable center positions
    centerLength = 0.8;
    centerAngle = M_PI/2;
    lLegLen.ang = lLegLen.ang + centerLength;
    lLegAng.ang = lLegAng.ang + centerAngle;
    rLegLen.ang = -rLegLen.ang + centerLength;
    rLegLen.vel = -rLegLen.vel;
    rLegAng.ang = -rLegAng.ang + centerAngle;
    rLegAng.vel = -rLegAng.vel;

    // Transform to motor positions and velocities
    lMotorAngle = legToMotorPos(lLegAng.ang, lLegLen.ang);
    lMotorVelocity = legToMotorVel(lLegAng, lLegLen);
    rMotorAngle = legToMotorPos(rLegAng.ang, rLegLen.ang);
    rMotorVelocity = legToMotorVel(rLegAng, rLegLen);
    // lMotorAngle.A / .B

    // Set the PD gains
    // lLeg
    // motorA
    P0.set(guiIn.p_gain);
    D0.set(guiIn.d_gain);
    // motorB
    P1.set(guiIn.p_gain);
    D1.set(guiIn.d_gain);
    // rLeg
    // motorA
    P2.set(guiIn.p_gain);
    D2.set(guiIn.d_gain);
    // motorB
    P3.set(guiIn.p_gain);
    D3.set(guiIn.d_gain);

    // lLeg
    // MotorA
    targetPos = lMotorAngle.A;
    currentPos = rs.lLeg.halfA.motorAngle;
    targetVel = lMotorVelocity.A;
    currentVel = rs.lLeg.halfA.motorVelocity;
    co.lLeg.motorCurrentA = pd0Controller(targetPos, currentPos, targetVel, currentVel);

    // MotorB
    targetPos = lMotorAngle.B;
    currentPos = rs.lLeg.halfB.motorAngle;
    targetVel = lMotorVelocity.B;
    currentVel = rs.lLeg.halfB.motorVelocity;
    co.lLeg.motorCurrentB = pd1Controller(targetPos, currentPos, targetVel, currentVel);

    // rLeg
    // MotorA
    targetPos = rMotorAngle.A;
    currentPos = rs.rLeg.halfA.motorAngle;
    targetVel = rMotorVelocity.A;
    currentVel = rs.rLeg.halfA.motorVelocity;
    co.rLeg.motorCurrentA = pd2Controller(targetPos, currentPos, targetVel, currentVel);

    // MotorB
    targetPos = rMotorAngle.B;
    currentPos = rs.rLeg.halfB.motorAngle;
    targetVel = rMotorVelocity.B;
    currentVel = rs.rLeg.halfB.motorVelocity;
    co.rLeg.motorCurrentB = pd3Controller(targetPos, currentPos, targetVel, currentVel);

    co.command = medulla_state_run;

    return co;
}

// Don't put control code below here!
bool ATCLegSinWave::configureHook() {
    // Connect to the subcontrollers
    // Sin controllers
    sin0 = this->getPeer(sin0Name);
    if (sin0)
        sin0Controller = sin0->provides("sinGen")->getOperation("runController");

    sin1 = this->getPeer(sin1Name);
    if (sin1)
        sin1Controller = sin1->provides("sinGen")->getOperation("runController");

    sin2 = this->getPeer(sin2Name);
    if (sin2)
        sin2Controller = sin2->provides("sinGen")->getOperation("runController");

    sin3 = this->getPeer(sin3Name);
    if (sin3)
        sin3Controller = sin3->provides("sinGen")->getOperation("runController");

    // Transforms
    legToMotorPos = this->provides("legToMotorTransforms")->getOperation("posTransform");
    legToMotorVel = this->provides("legToMotorTransforms")->getOperation("velTransform");

    // PD controllers
    pd0 = this->getPeer(pd0Name);
    if (pd0)
        pd0Controller = pd0->provides("pd")->getOperation("runController");

    pd1 = this->getPeer(pd1Name);
    if (pd1)
        pd1Controller = pd1->provides("pd")->getOperation("runController");

    pd2 = this->getPeer(pd2Name);
    if (pd2)
        pd2Controller = pd2->provides("pd")->getOperation("runController");

    pd3 = this->getPeer(pd3Name);
    if (pd3)
        pd3Controller = pd3->provides("pd")->getOperation("runController");

    // Get references to the attributes
    P0 = pd0->properties()->getProperty("P");
    D0 = pd0->properties()->getProperty("D");
    P1 = pd1->properties()->getProperty("P");
    D1 = pd1->properties()->getProperty("D");
    P2 = pd2->properties()->getProperty("P");
    D2 = pd2->properties()->getProperty("D");
    P3 = pd3->properties()->getProperty("P");
    D3 = pd3->properties()->getProperty("D");

    log(Info) << "[ATCLSW] configured!" << endlog();
    return true;
}

bool ATCLegSinWave::startHook() {
    log(Info) << "[ATCLSW] started!" << endlog();
    return true;
}

void ATCLegSinWave::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCLegSinWave::stopHook() {
    log(Info) << "[ATCLSW] stopped!" << endlog();
}

void ATCLegSinWave::cleanupHook() {
    log(Info) << "[ATCLSW] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCLegSinWave)

}
}

