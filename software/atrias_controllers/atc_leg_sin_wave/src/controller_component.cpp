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
    this->addProperty("sin0Name", sin0Name)
        .doc("Name of 0th sin generator subcontroller.");
    this->addProperty("sin1Name", sin1Name)
        .doc("Name of 1st sin generator subcontroller.");

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
        controllerOutput.lLeg.motorCurrentA = 0.0;
        controllerOutput.lLeg.motorCurrentB = 0.0;
        return controllerOutput;
    }

    // Get a sinusoidal input
    legLen = sin0Controller(guiIn.leg_len_frq, guiIn.leg_len_amp);
    legAng = sin1Controller(guiIn.leg_ang_frq, guiIn.leg_ang_amp);
    // legLen.pos / .vel

    // Set resonable center positions
    centerLength = 0.85;
    centerAngle = M_PI/2;
    legLen.pos = legLen.pos + centerLength;
    legAng.pos = legAng.pos + centerAngle;

    // Transform to motor positions and velocities
    motorAngle = legToMotorPos(legAng.pos, legLen.pos);
    motorVelocity = legToMotorVel(legAng, legLen);
    // motorAngle.A / .B

    // Set the PD gains
    // motorA
    P0.set(guiIn.p_gain);
    D0.set(guiIn.d_gain);
    // motorB
    P1.set(guiIn.p_gain);
    D1.set(guiIn.d_gain);

    // Calculate motorA current
    targetPos = motorAngle.A;
    currentPos = rs.lLeg.halfA.motorAngle;
    targetVel = motorVelocity.A;
    currentVel = rs.lLeg.halfA.motorVelocity;
    controllerOutput.lLeg.motorCurrentA = pd0Controller(targetPos, currentPos, targetVel, currentVel);

    // Calculate motorB current
    targetPos = motorAngle.B;
    currentPos = rs.lLeg.halfB.motorAngle;
    targetVel = motorVelocity.B;
    currentVel = rs.lLeg.halfB.motorVelocity;
    controllerOutput.lLeg.motorCurrentB = pd1Controller(targetPos, currentPos, targetVel, currentVel);

    return controllerOutput;
}

// Don't put control code below here!
bool ATCLegSinWave::configureHook() {
    // Connect to the subcontrollers
    // Sin controllers
    sin0 = this->getPeer(sin0Name);
    if (sin0)
        sin0Controller = sin0->provides("sg")->getOperation("runController");
    sin1 = this->getPeer(sin1Name);
    if (sin1)
        sin1Controller = sin1->provides("sg")->getOperation("runController");

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

    // Get references to the attributes
    P0 = pd0->properties()->getProperty("P");
    D0 = pd0->properties()->getProperty("D");
    P1 = pd1->properties()->getProperty("P");
    D1 = pd1->properties()->getProperty("D");

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

