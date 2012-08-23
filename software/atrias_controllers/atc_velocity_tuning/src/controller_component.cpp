/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_velocity_tuning controller.
 */

#include <atc_velocity_tuning/controller_component.h>

namespace atrias {
namespace controller {

ATCVelocityTuning::ATCVelocityTuning(std::string name):
    RTT::TaskContext(name),
    guiDataOut("gui_data_out"),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCVelocityTuning::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add ports
    addEventPort(guiDataIn);
    addPort(guiDataOut);

    pubTimer = new GuiPublishTimer(20);

    log(Info) << "[ATCMT] atc_velocity_tuning controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCVelocityTuning::runController(atrias_msgs::robot_state rs) {
    // Stuff the msg
    controllerOutput.lLeg.motorCurrentA = guiIn.des_motor_torque_A;
    controllerOutput.lLeg.motorCurrentB = guiIn.des_motor_torque_B;
    controllerOutput.lLeg.motorCurrentHip = guiIn.des_motor_torque_hip;

    // Command a run state
    controllerOutput.command = medulla_state_run;

    // Send data to the GUI
    if (pubTimer->readyToSend())
        guiDataOut.write(guiOut);

    // Output for RTOps
    return controllerOutput;
}

// Don't put control code below here!
bool ATCVelocityTuning::configureHook() {

    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
}

bool ATCVelocityTuning::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCVelocityTuning::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCVelocityTuning::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCVelocityTuning::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCVelocityTuning)

}
}
