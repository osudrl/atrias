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
    td = 0.0;
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
        if (td < (AMC_PEAK_TIME + AMC_FOLDBACK_TIME)) {
            td += 1.0;   // Increment td if under 12s.
        }
        if (td > AMC_PEAK_TIME) {
            foldbackTriggered = true;   // We're in foldback period if counter is past peak current duration. TODO: does this stay triggered too long for peak time at end?
        }
    }
    else {
        if (td > 0) {
            td -= (AMC_IC-co.rLeg.motorCurrentA)/(AMC_IP-AMC_IC)/2.0;   // Count down such that A2 = 2*A1 (see datasheet)
        }
        foldbackTriggered = false;   // Disable foldback.
    }

    // Estimate current limit
    if (foldbackTriggered) {
        if (curLimit > AMC_IC) {
            curLimit -= (AMC_IP-AMC_IC)/AMC_FOLDBACK_TIME;   // In foldback period, gradually decrease current limit to AMC_IC.
        }
    }
    else {
        if (curLimit < AMC_IP) {
            curLimit += (AMC_IP - curLimit) / (td * AMC_IP/AMC_IC - AMC_PEAK_TIME);   // Otherwise, increase current fast enough to leave peak time at the end.
	  }
    }

    // Set current limit if GUI says so
    if (guiIn.limitCurrent) {
        if (co.rLeg.motorCurrentA > curLimit) {
            co.rLeg.motorCurrentA = curLimit;
        }
    }

    log(Info) << "[ATCMT] td: " << td << "  fT: " << foldbackTriggered << "  cL: " << curLimit << endlog();

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
