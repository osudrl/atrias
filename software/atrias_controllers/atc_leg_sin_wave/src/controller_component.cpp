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
    this->addProperty("pd4Name", pd4Name)
        .doc("Name of 4th PD subcontroller.");
    this->addProperty("pd5Name", pd5Name)
        .doc("Name of 5th PD subcontroller.");
    this->addProperty("sin0Name", sin0Name)
        .doc("Name of 0th sin generator subcontroller.");
    this->addProperty("sin1Name", sin1Name)
        .doc("Name of 1st sin generator subcontroller.");
    this->addProperty("sin2Name", sin2Name)
        .doc("Name of 2th sin generator subcontroller.");
    this->addProperty("sin3Name", sin3Name)
        .doc("Name of 3th sin generator subcontroller.");
    this->addProperty("sin4Name", sin4Name)
        .doc("Name of 4th sin generator subcontroller.");
    this->addProperty("sin5Name", sin5Name)
        .doc("Name of 5th sin generator subcontroller.");

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

    // Set the sin controller phase shifts for the hip
    // so they start by going inwards
    sin4SetPhase(3.0/4.0/guiIn.hip_frq);
    sin5SetPhase(1.0/4.0/guiIn.hip_frq);

    // Get a sinusoidal input
    lLegLen = sin0Controller(guiIn.leg_len_frq, guiIn.leg_len_amp);
    lLegAng = sin1Controller(guiIn.leg_ang_frq, guiIn.leg_ang_amp);
    rLegLen = sin2Controller(guiIn.leg_len_frq, guiIn.leg_len_amp);
    rLegAng = sin3Controller(guiIn.leg_ang_frq, guiIn.leg_ang_amp);
    lHip    = sin4Controller(guiIn.hip_frq, guiIn.hip_amp);
    rHip    = sin5Controller(guiIn.hip_frq, guiIn.hip_amp);

    // Set resonable center positions
    centerLength = 0.8;
    centerAngle = M_PI/2.0;
    static double lHipStart = rs.lLeg.hip.legBodyAngle;
    static double rHipStart = rs.rLeg.hip.legBodyAngle;

    lLegLen.ang = lLegLen.ang + centerLength;

    lLegAng.ang = lLegAng.ang + centerAngle;

    rLegLen.ang = -rLegLen.ang + centerLength;
    rLegLen.vel = -rLegLen.vel;

    rLegAng.ang = -rLegAng.ang + centerAngle;
    rLegAng.vel = -rLegAng.vel;

  //angle       = restPos   + sinWave  + vertical offset
    lHip.ang    = lHipStart + lHip.ang + guiIn.hip_amp;
    rHip.ang    = rHipStart + rHip.ang - guiIn.hip_amp;


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
    // hip
    P2.set(guiIn.hip_p_gain);
    D2.set(guiIn.hip_d_gain);
    // rLeg
    // motorA
    P3.set(guiIn.p_gain);
    D3.set(guiIn.d_gain);
    // motorB
    P4.set(guiIn.p_gain);
    D4.set(guiIn.d_gain);
    // hip
    P5.set(guiIn.hip_p_gain);
    D5.set(guiIn.hip_d_gain);

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

    // Hip
    targetPos = lHip.ang;
    currentPos = rs.lLeg.hip.legBodyAngle;
    targetVel = lHip.vel;
    currentVel = rs.lLeg.hip.legBodyVelocity;
    co.lLeg.motorCurrentHip = pd2Controller(targetPos, currentPos, targetVel, currentVel);

    // rLeg
    // MotorA
    targetPos = rMotorAngle.A;
    currentPos = rs.rLeg.halfA.motorAngle;
    targetVel = rMotorVelocity.A;
    currentVel = rs.rLeg.halfA.motorVelocity;
    co.rLeg.motorCurrentA = pd3Controller(targetPos, currentPos, targetVel, currentVel);

    // MotorB
    targetPos = rMotorAngle.B;
    currentPos = rs.rLeg.halfB.motorAngle;
    targetVel = rMotorVelocity.B;
    currentVel = rs.rLeg.halfB.motorVelocity;
    co.rLeg.motorCurrentB = pd4Controller(targetPos, currentPos, targetVel, currentVel);

    // Hip
    targetPos = rHip.ang;
    currentPos = rs.rLeg.hip.legBodyAngle;
    targetVel = rHip.vel;
    currentVel = rs.rLeg.hip.legBodyVelocity;
    co.rLeg.motorCurrentHip = pd5Controller(targetPos, currentPos, targetVel, currentVel);

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

    sin4 = this->getPeer(sin4Name);
    if (sin4)
    {
        sin4Controller = sin4->provides("sinGen")->getOperation("runController");
        sin4SetPhase   = sin4->provides("sinGen")->getOperation("setPhase");
    }

    sin5 = this->getPeer(sin5Name);
    if (sin5)
    {
        sin5Controller = sin5->provides("sinGen")->getOperation("runController");
        sin5SetPhase   = sin5->provides("sinGen")->getOperation("setPhase");
    }

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

    pd4 = this->getPeer(pd4Name);
    if (pd4)
        pd4Controller = pd4->provides("pd")->getOperation("runController");

    pd5 = this->getPeer(pd5Name);
    if (pd5)
        pd5Controller = pd5->provides("pd")->getOperation("runController");

    // Get references to the attributes
    P0 = pd0->properties()->getProperty("P");
    D0 = pd0->properties()->getProperty("D");
    P1 = pd1->properties()->getProperty("P");
    D1 = pd1->properties()->getProperty("D");
    P2 = pd2->properties()->getProperty("P");
    D2 = pd2->properties()->getProperty("D");
    P3 = pd3->properties()->getProperty("P");
    D3 = pd3->properties()->getProperty("D");
    P4 = pd4->properties()->getProperty("P");
    D4 = pd4->properties()->getProperty("D");
    P5 = pd5->properties()->getProperty("P");
    D5 = pd5->properties()->getProperty("D");

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

