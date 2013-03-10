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

    curLimit = AMC_IP;
    td = 0;
    dd = 0;
    foldbackTriggered = false;

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

    // Do we think the AMC amplifiers are in foldback mode?
    if (co.rLeg.motorCurrentA > AMC_IC) {
        if (td < 12000) {
            td++;   // Increment td if under 12s.
        }
        if (td > 2000) {
            foldbackTriggered = true;   // Enable foldback if current peaks for 2s.
        }
    }
    else {
        if (td > 0) {
            dd = (dd+1) % 2;
            td -= dd;   // Count down every other loop so A2 = 2*A1 (see datasheet)
        }
        else {
            foldbackTriggered = false;   // Reset foldback if A2 has been filled.
        }
    }

    // Estimate current limit
    if (foldbackTriggered) {
        if (curLimit > AMC_IC) {
            curLimit -= (AMC_IP-AMC_IC)/10000;   // In foldback period, decrease current limit to AMC_IC over the course of 10s.
        }
    }
    else {
        curLimit = AMC_IP;   // Otherwise, reset current limit to max.
    }

    // Set current limit if GUI says so
    if (guiIn.limitCurrent) {
        if (co.rLeg.motorCurrentA > curLimit) {
            co.rLeg.motorCurrentA = curLimit;
        }
    }

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
