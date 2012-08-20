/*! \file controller_component.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Orocos Component code for the motor position controller.
 */

#include <atc_motor_position/controller_component.h>

namespace atrias {
namespace controller {

ATCMotorPosition::ATCMotorPosition(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in"),
    guiDataOut("gui_data_out")
{
    this->provides("atc")
        ->addOperation("runController", &ATCMotorPosition::runController, this, OwnThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add properties.
    this->addProperty("pd0Name", pd0Name)
        .doc("Name of 0th PD subcontroller.");
    this->addProperty("pd1Name", pd1Name)
        .doc("Name of 1st PD subcontroller.");

    // Add ports.
    addEventPort(guiDataIn);
    addPort(guiDataOut);

    pubTimer = new GuiPublishTimer(20);

    log(Info) << "[ATCMP] Motor position controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCMotorPosition::runController(atrias_msgs::robot_state rs) {
    // Set the PD gains
    // MotorA
    P0.set(10.0);
    D0.set(1.0);
    // MotorB
    P1.set(10.0);
    D1.set(1.0);

    // Calculate motorA output
    targetPos = guiIn.des_motor_ang_A;
    currentPos = rs.lLeg.halfA.motorAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.halfA.motorVelocity;
    controllerOutput.lLeg.motorCurrentA = pd0Controller(targetPos, currentPos, targetVel, currentVel);

    // Calculate motorB output
    targetPos = guiIn.des_motor_ang_B;
    currentPos = rs.lLeg.halfB.motorAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.halfB.motorVelocity;
    controllerOutput.lLeg.motorCurrentB = pd1Controller(targetPos, currentPos, targetVel, currentVel);

    // Send data to the GUI
    if (pubTimer->readyToSend())
        guiDataOut.write(guiOut);

    return controllerOutput;
}

// Don't put control code below here!
bool ATCMotorPosition::configureHook() {
    // Connect to the subcontrollers
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

    log(Info) << "[ATCMP] configured!" << endlog();
    return true;
}

bool ATCMotorPosition::startHook() {
    log(Info) << "[ATCMP] started!" << endlog();
    return true;
}

void ATCMotorPosition::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCMotorPosition::stopHook() {
    log(Info) << "[ATCMP] stopped!" << endlog();
}

void ATCMotorPosition::cleanupHook() {
    delete pubTimer;
    log(Info) << "[ATCMP] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCMotorPosition)

}
}

