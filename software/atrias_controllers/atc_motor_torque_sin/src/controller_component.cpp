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
        ->addOperation("runController", &ATCMotorTorqueSin::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add properties.
    this->addProperty("sin0Name", sin0Name)
        .doc("Name of 0th sin generator subcontroller.");

    // Add ports.
    addEventPort(guiDataIn);

    // Set default values
    prevMotor = 6;

    log(Info) << "[ATCMTS] Motor sin wave controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCMotorTorqueSin::runController(atrias_msgs::robot_state rs) {
    // Do nothing unless told otherwise
    co.lLeg.motorCurrentA   = 0.0;
    co.lLeg.motorCurrentB   = 0.0;
    co.lLeg.motorCurrentHip = 0.0;
    co.rLeg.motorCurrentA   = 0.0;
    co.rLeg.motorCurrentB   = 0.0;
    co.rLeg.motorCurrentHip = 0.0;

    // Only run the controller when we're enabled
    if ((uint8_t)rs.cmState != (uint8_t)controllerManager::RtOpsCommand::ENABLE)
        return co;

    // Check to see if this is a new motor
    // If so, reset the sin wave so it starts at zero amps
    if (prevMotor != guiIn.motor)
        sin0Reset();

    prevMotor = guiIn.motor;

    // Get a sinusoidal input
    sinOut = sin0Controller(guiIn.frq, guiIn.amp);

    // Compensate for motor construction and assembly variations 
    current = sinOut.ang + guiIn.offset;

    // lLeg A
    if      (guiIn.motor == 0)
        co.lLeg.motorCurrentA   = current;
    // lLeg B
    else if (guiIn.motor == 1)
        co.lLeg.motorCurrentB   = current;
    // lLeg Hip
    else if (guiIn.motor == 2)
        co.lLeg.motorCurrentHip = current;
    // rLeg A
    else if (guiIn.motor == 3)
        co.rLeg.motorCurrentA   = current;
    // rLeg B
    else if (guiIn.motor == 4)
        co.rLeg.motorCurrentB   = current;
    // rLeg Hip
    else if (guiIn.motor == 5)
        co.rLeg.motorCurrentHip = current;

    co.command = medulla_state_run;

    return co;
}

// Don't put control code below here!
bool ATCMotorTorqueSin::configureHook() {
    // Connect to the sin controller operations
    sin0 = this->getPeer(sin0Name);
    if (sin0)
    {
        sin0Controller = sin0->provides("sinGen")->getOperation("runController");
        sin0Reset      = sin0->provides("sinGen")->getOperation("reset");
    }

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

