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
    curCounter = COUNTER_MAX;
    timeSinceFB = 0.0;
    inFoldback = false;

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

    // Run current limit estimator.
    estimateCurrentLimit();

    // Set current limit if GUI says so
    if (guiIn.limitCurrent) {
        if (co.rLeg.motorCurrentA > curLimit) {
            co.rLeg.motorCurrentA = curLimit;
        }
    }

    log(Info) << "[ATCMT] timeSinceFB: " << timeSinceFB << "  In Foldback: " << inFoldback << "  curCounter: " << curCounter << "  cL: " << curLimit << endlog();

    // Output for RTOps
    return co;
}

// Don't put control code below here!

void ATCMotorTorque::estimateCurrentLimit()
{
    if (!inFoldback && co.rLeg.motorCurrentA > AMC_IC) {
        inFoldback = true;

        // Reset counter to current limit.
        curCounter = curLimit;

        // Reset time since foldback.
        timeSinceFB = 0.0;
    }
    else if (inFoldback && co.rLeg.motorCurrentA <= AMC_IC) {
        inFoldback = false;
    }

    // Update counter.
    if (inFoldback) {
        // Decrement counter at constant slope regardless of target current.
        if (curCounter > AMC_IC) {
            curCounter -= M_FB / 1000;
        }
        else {
            curCounter = AMC_IC;
        }
    }
    else {
        // Increment counter based on target current.
        if (curCounter < COUNTER_MAX) {
            curCounter += M_FB * (AMC_IC-co.rLeg.motorCurrentA) / (AMC_IP-AMC_IC) / 2 / 1000;
        }
        else {
            curCounter = COUNTER_MAX;
        }

        // Set current limit to minimum among counter, recovery rate cap, and
        // peak current limit.
        curLimit = MIN(curCounter, MIN(AMC_IC+M_FB*timeSinceFB, AMC_IP));

        // Increment timer.
        timeSinceFB += 0.001;
    }
}

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
