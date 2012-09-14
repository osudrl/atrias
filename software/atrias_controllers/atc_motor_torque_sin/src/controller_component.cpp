/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_motor_torque_sin controller
 */

#include <atc_motor_torque_sin/controller_component.h>

namespace atrias {
namespace controller {

ATCMotorTorqueSin::ATCMotorTorqueSin(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCMotorTorqueSin::runController, this, OwnThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add properties.
    this->addProperty("sin0Name", sin0Name)
        .doc("Name of 0th sin generator subcontroller.");

    // Add ports.
    addEventPort(guiDataIn);

    log(Info) << "[ATCMTS] Motor sin wave controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCMotorTorqueSin::runController(atrias_msgs::robot_state rs) {
    // Only run the controller when we're enabled
    if ((uint8_t)rs.cmState != (uint8_t)controllerManager::RtOpsCommand::ENABLE)
    {
        // Do nothing
        co.lLeg.motorCurrentA   = 0.0;
        co.lLeg.motorCurrentB   = 0.0;
        co.lLeg.motorCurrentHip = 0.0;
        co.rLeg.motorCurrentA   = 0.0;
        co.rLeg.motorCurrentB   = 0.0;
        co.rLeg.motorCurrentHip = 0.0;
        return co;
    }

    // Get a sinusoidal input
    sinOut = sin0Controller(guiIn.frq, guiIn.amp);

    current = sinOut.pos + guiIn.offset;

    // lLeg A
    if      (guiIn.motor == 1)
        co.lLeg.motorCurrentA   = current;
    // lLeg B
    else if (guiIn.motor == 2)
        co.lLeg.motorCurrentB   = current;
    // lLeg Hip
    else if (guiIn.motor == 3)
        co.lLeg.motorCurrentHip = current;
    // rLeg A
    else if (guiIn.motor == 4)
        co.rLeg.motorCurrentA   = current;
    // rLeg B
    else if (guiIn.motor == 5)
        co.rLeg.motorCurrentB   = current;
    // rLeg Hip
    else if (guiIn.motor == 6)
        co.rLeg.motorCurrentHip = current;

    co.command = medulla_state_run;

    return co;
}

// Don't put control code below here!
bool ATCMotorTorqueSin::configureHook() {
    // Connect to the subcontrollers
    // Sin controllers
    sin0 = this->getPeer(sin0Name);
    if (sin0)
        sin0Controller = sin0->provides("sg")->getOperation("runController");

    log(Info) << "[ATCMTS] configured!" << endlog();
    return true;
}

bool ATCMotorTorqueSin::startHook() {
    log(Info) << "[ATCMTS] started!" << endlog();
    return true;
}

void ATCMotorTorqueSin::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCMotorTorqueSin::stopHook() {
    log(Info) << "[ATCMTS] stopped!" << endlog();
}

void ATCMotorTorqueSin::cleanupHook() {
    log(Info) << "[ATCMTS] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCMotorTorqueSin)

}
}

