/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the motor sin wave controller
 */

#include <atc_motor_sin_wave/controller_component.h>

namespace atrias {
namespace controller {

ATCMotorSinWave::ATCMotorSinWave(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCMotorSinWave::runController, this, OwnThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add properties.
    this->addProperty("pd0Name", pd0Name)
        .doc("Name of 0th PD subcontroller.");
    this->addProperty("pd1Name", pd1Name)
        .doc("Name of 1st PD subcontroller.");
    this->addProperty("sin0Name", sin0Name)
        .doc("Name of 0th sin generator subcontroller.");

    // Add ports.
    addEventPort(guiDataIn);

    // Set Defaults
    targetPos = 0;
    currentPos = 0;
    targetVel = 0;
    currentVel = 0;

    log(Info) << "[ATCMSW] Motor sin wave controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCMotorSinWave::runController(atrias_msgs::robot_state rs) {
    // Only run the controller when we're enabled
    if ((uint8_t)rs.cmState != (uint8_t)controllerManager::RtOpsCommand::ENABLE)
    {
        // Do nothing
        controllerOutput.lLeg.motorCurrentA = 0.0;
        controllerOutput.lLeg.motorCurrentB = 0.0;
        return controllerOutput;
    }

    // Get a sinusoidal input
    motorBSin = sin0Controller(guiIn.leg_ang_frq, guiIn.leg_ang_amp);
    // motorBSin.pos / .vel

    // Set resonable center positions
    centerAAngle = M_PI/4;
    centerBAngle = 3*M_PI/4;
    motorBSin.pos = motorBSin.pos + centerBAngle;

    // Set the PD gains
    // motorA
    P0.set(guiIn.p_gain);
    D0.set(guiIn.d_gain);
    // motorB
    P1.set(guiIn.p_gain);
    D1.set(guiIn.d_gain);

    // Calculate motorA current
    targetPos = centerAAngle;
    currentPos = rs.lLeg.halfA.motorAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.halfA.motorVelocity;
    controllerOutput.lLeg.motorCurrentA = pd0Controller(targetPos, currentPos, targetVel, currentVel);
    
    // Calculate motorB current
    targetPos = motorBSin.pos;
    currentPos = rs.lLeg.halfB.motorAngle;
    targetVel = motorBSin.vel;
    currentVel = rs.lLeg.halfB.motorVelocity;
    controllerOutput.lLeg.motorCurrentB = pd1Controller(targetPos, currentPos, targetVel, currentVel);

    controllerOutput.command = medulla_state_run;

    return controllerOutput;
}

// Don't put control code below here!
bool ATCMotorSinWave::configureHook() {
    // Connect to the subcontrollers
    // Sin controllers
    sin0 = this->getPeer(sin0Name);
    if (sin0)
        sin0Controller = sin0->provides("sg")->getOperation("runController");

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

    log(Info) << "[ATCMSW] configured!" << endlog();
    return true;
}

bool ATCMotorSinWave::startHook() {
    log(Info) << "[ATCMSW] started!" << endlog();
    return true;
}

void ATCMotorSinWave::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCMotorSinWave::stopHook() {
    log(Info) << "[ATCMSW] stopped!" << endlog();
}

void ATCMotorSinWave::cleanupHook() {
    log(Info) << "[ATCMSW] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCMotorSinWave)

}
}

