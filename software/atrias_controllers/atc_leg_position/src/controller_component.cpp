/*! \file controller_component.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Orocos Component code for the leg position controller.
 */

#include <atc_leg_position/controller_component.h>

namespace atrias {
namespace controller {

ATCLegPosition::ATCLegPosition(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in"),
    guiDataOut("gui_data_out")
{
    this->provides("atc")
        ->addOperation("runController", &ATCLegPosition::runController, this, OwnThread)
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

    // Set Defaults
    targetPos = 0;
    currentPos = 0;
    targetVel = 0;
    currentVel = 0;

    log(Info) << "[ATCMP] Leg position controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCLegPosition::runController(atrias_msgs::robot_state rs) {
    // Set the PD gains
    // LegA
    P0.set(guiIn.p_gain);
    D0.set(guiIn.d_gain);
    // LegB
    P1.set(guiIn.p_gain);
    D1.set(guiIn.d_gain);

    // Transform from leg angle and length to motor positions
    motorAngle = legToMotorPos(guiIn.leg_ang, guiIn.leg_len);

    // Calculate motorA current
    targetPos = motorAngle.A;
    currentPos = rs.lLeg.halfA.motorAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.halfA.motorVelocity;
    controllerOutput.lLeg.motorCurrentA = pd0Controller(targetPos, currentPos, targetVel, currentVel);

    // Calculate motorB current
    targetPos = motorAngle.B;
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
bool ATCLegPosition::configureHook() {
    // Connect to the subcontrollers
    pd0 = this->getPeer(pd0Name);
    if (pd0)
        pd0Controller = pd0->provides("pd")->getOperation("runController");

    pd1 = this->getPeer(pd1Name);
    if (pd1)
        pd1Controller = pd1->provides("pd")->getOperation("runController");

    legToMotorPos = this->provides("legToMotorTransforms")->getOperation("posTransform");

    // Get references to the attributes
    P0 = pd0->properties()->getProperty("P");
    D0 = pd0->properties()->getProperty("D");
    P1 = pd1->properties()->getProperty("P");
    D1 = pd1->properties()->getProperty("D");

    log(Info) << "[ATCMP] configured!" << endlog();
    return true;
}

bool ATCLegPosition::startHook() {
    log(Info) << "[ATCMP] started!" << endlog();
    return true;
}

void ATCLegPosition::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCLegPosition::stopHook() {
    log(Info) << "[ATCMP] stopped!" << endlog();
}

void ATCLegPosition::cleanupHook() {
    delete pubTimer;
    log(Info) << "[ATCMP] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCLegPosition)

}
}

