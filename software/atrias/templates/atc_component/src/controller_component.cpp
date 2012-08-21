/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_template controller.
 */

#include <atc_template/controller_component.h>

namespace atrias {
namespace controller {

ATCTemplate::ATCTemplate(std::string name):
    RTT::TaskContext(name),
		guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCTemplate::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    addEventPort(guiDataIn);

    log(Info) << "[ATCMT] atc_template controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCTemplate::runController(atrias_msgs::robot_state rs) {
    // Stuff the msg
    controllerOutput.lLeg.motorCurrentA = guiIn.des_motor_torque_A;
    controllerOutput.lLeg.motorCurrentB = guiIn.des_motor_torque_B;
    controllerOutput.lLeg.motorCurrentHip = guiIn.des_motor_torque_hip;

    // Output for RTOps
    return controllerOutput;
}

// Don't put control code below here!
bool ATCTemplate::configureHook() {
    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
}

bool ATCTemplate::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCTemplate::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCTemplate::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCTemplate::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCTemplate)

}
}
