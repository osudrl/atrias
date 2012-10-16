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
        ->addOperation("runController", &ATCLegSinWave::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add subcontroller names
    this->addProperty("pd0Name",  pd0Name);
    this->addProperty("pd1Name",  pd1Name);
    this->addProperty("pd2Name",  pd2Name);
    this->addProperty("pd3Name",  pd3Name);
    this->addProperty("pd4Name",  pd4Name);
    this->addProperty("pd5Name",  pd5Name);
    this->addProperty("sin0Name", sin0Name);
    this->addProperty("sin1Name", sin1Name);
    this->addProperty("sin2Name", sin2Name);
    this->addProperty("sin3Name", sin3Name);
    this->addProperty("sin4Name", sin4Name);
    this->addProperty("sin5Name", sin5Name);

    // Add ports
    addEventPort(guiDataIn);

    // Set Defaults
    targetPos  = 0.0;
    currentPos = 0.0;
    targetVel  = 0.0;
    currentVel = 0.0;
    hipPeriod  = 1.0;

    log(Info) << "[ATCLSW] Leg sin wave controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCLegSinWave::runController(atrias_msgs::robot_state rs) {
    // Default to zero
    co.lLeg.motorCurrentA   = 0.0;
    co.lLeg.motorCurrentB   = 0.0;
    co.lLeg.motorCurrentHip = 0.0;
    co.rLeg.motorCurrentA   = 0.0;
    co.rLeg.motorCurrentB   = 0.0;
    co.rLeg.motorCurrentHip = 0.0;

    // Only run the controller when we're enabled
    if ((uint8_t)rs.cmState != (uint8_t)controllerManager::RtOpsCommand::ENABLE)
    {
        // Get hip starting positions
        lHipStart = rs.lLeg.hip.legBodyAngle;
        rHipStart = rs.rLeg.hip.legBodyAngle;
        // Reset the sin controllers
        sin0Reset();
        sin1Reset();
        sin2Reset();
        sin3Reset();
        sin4Reset();
        sin5Reset();
        return co;
    }

    // If the frequency doesn't make the period infinite
    if (guiIn.hip_frq > 0.001)
        hipPeriod = 1.0/guiIn.hip_frq;

    // Set the sin controller phase shifts for the hip
    // so they start by going inwards
    sin4SetPhase(3.0/4.0*hipPeriod);
    sin5SetPhase(1.0/4.0*hipPeriod);

    // Get a sinusoidal input
    lLegLen = sin0Controller(guiIn.leg_len_frq, guiIn.leg_len_amp);
    lLegAng = sin1Controller(guiIn.leg_ang_frq, guiIn.leg_ang_amp);
    rLegLen = sin2Controller(guiIn.leg_len_frq, guiIn.leg_len_amp);
    rLegAng = sin3Controller(guiIn.leg_ang_frq, guiIn.leg_ang_amp);
    lHip    = sin4Controller(guiIn.hip_frq,     guiIn.hip_amp);
    rHip    = sin5Controller(guiIn.hip_frq,     guiIn.hip_amp);

    // Set resonable center positions
    centerLength = 0.8;
    centerAngle = M_PI/2.0;

    lLegLen.ang = lLegLen.ang + centerLength;

    lLegAng.ang = lLegAng.ang + centerAngle;

    rLegLen.ang = -rLegLen.ang + centerLength;
    rLegLen.vel = -rLegLen.vel;

    rLegAng.ang = -rLegAng.ang + centerAngle;
    rLegAng.vel = -rLegAng.vel;


  //angle       = restPos   +  sinWave  + vertical offset
    lHip.ang    = lHipStart + (lHip.ang + guiIn.hip_amp);
    rHip.ang    = rHipStart + (rHip.ang - guiIn.hip_amp);

    // Enforce reasonable hip angles
    vertical = 3.0*M_PI/2.0;
    inAngle  = M_PI/180.0*10.0;
    outAngle = M_PI/180.0*20.0;
    if (lHip.ang < (vertical - inAngle))
        lHip.ang =  vertical - inAngle;
    if (lHip.ang > (vertical + outAngle))
        lHip.ang =  vertical + outAngle;
    if (rHip.ang > (vertical + inAngle))
        rHip.ang =  vertical + inAngle;
    if (rHip.ang < (vertical - outAngle))
        rHip.ang =  vertical - outAngle;

    // Transform to motor positions and velocities
    lMotorAngle    = legToMotorPos(lLegAng.ang, lLegLen.ang);
    lMotorVelocity = legToMotorVel(lLegAng, lLegLen);
    rMotorAngle    = legToMotorPos(rLegAng.ang, rLegLen.ang);
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
    {
        sin0Controller = sin0->provides("sinGen")->getOperation("runController");
        sin0Reset      = sin0->provides("sinGen")->getOperation("reset");
    }

    sin1 = this->getPeer(sin1Name);
    if (sin1)
    {
        sin1Controller = sin1->provides("sinGen")->getOperation("runController");
        sin1Reset      = sin1->provides("sinGen")->getOperation("reset");
    }

    sin2 = this->getPeer(sin2Name);
    if (sin2)
    {
        sin2Controller = sin2->provides("sinGen")->getOperation("runController");
        sin2Reset      = sin2->provides("sinGen")->getOperation("reset");
    }

    sin3 = this->getPeer(sin3Name);
    if (sin3)
    {
        sin3Controller = sin3->provides("sinGen")->getOperation("runController");
        sin3Reset      = sin3->provides("sinGen")->getOperation("reset");
    }

    sin4 = this->getPeer(sin4Name);
    if (sin4)
    {
        sin4Controller = sin4->provides("sinGen")->getOperation("runController");
        sin4SetPhase   = sin4->provides("sinGen")->getOperation("setPhase");
        sin4Reset      = sin4->provides("sinGen")->getOperation("reset");
    }

    sin5 = this->getPeer(sin5Name);
    if (sin5)
    {
        sin5Controller = sin5->provides("sinGen")->getOperation("runController");
        sin5SetPhase   = sin5->provides("sinGen")->getOperation("setPhase");
        sin5Reset      = sin5->provides("sinGen")->getOperation("reset");
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

