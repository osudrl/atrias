/*! \file controller_component.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Orocos Component code for the motor torque controller.
 */

#include <atc_motor_torque/controller_component.h>

namespace atrias {
namespace controller {

ATCMotorTorque::ATCMotorTorque(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCMotorTorque::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    addEventPort(guiDataIn);

    log(Info) << "[ATCMT] Motor torque controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCMotorTorque::runController(atrias_msgs::robot_state rs) {
    // Stuff the msg
    co.lLeg.motorCurrentA = guiIn.des_motor_torque_left_A;
    co.lLeg.motorCurrentB = guiIn.des_motor_torque_left_B;
    co.lLeg.motorCurrentHip = guiIn.des_motor_torque_left_hip;
    co.rLeg.motorCurrentA = guiIn.des_motor_torque_right_A;
    co.rLeg.motorCurrentB = guiIn.des_motor_torque_right_B;
    co.rLeg.motorCurrentHip = guiIn.des_motor_torque_right_hip;
    co.command = medulla_state_run;

    // Output for RTOps
    return co;
}

// Don't put control code below here!
bool ATCMotorTorque::configureHook() {
    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
}

bool ATCMotorTorque::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCMotorTorque::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCMotorTorque::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCMotorTorque::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCMotorTorque)

}
}
