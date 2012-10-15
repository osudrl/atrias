/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the robot position controller.
 */

#include <asc_robot_pd/controller_component.h>

namespace atrias {
namespace controller {

ASCRobotPd::ASCRobotPd(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in"),
    guiDataOut("gui_data_out")
{
    this->provides("atc")
        ->addOperation("runController", &ASCRobotPd::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add properties.
    this->addProperty("pd0Name", pd0Name)
        .doc("Name of 0th PD subcontroller.");
    this->addProperty("pd1Name", pd1Name)
        .doc("Name of 1st PD subcontroller.");
    this->addProperty("pd2Name", pd2Name)
        .doc("Name of 2th PD subcontroller.");
    this->addProperty("pd3Name", pd3Name)
        .doc("Name of 3st PD subcontroller.");
    this->addProperty("pd4Name", pd4Name)
        .doc("Name of 4st PD subcontroller.");
    this->addProperty("pd5Name", pd5Name)
        .doc("Name of 5st PD subcontroller.");

    // Add ports.
    addEventPort(guiDataIn);
    addPort(guiDataOut);

    pubTimer = new GuiPublishTimer(20);

    // Set Defaults
    targetPos = 0;
    currentPos = 0;
    targetVel = 0;
    currentVel = 0;

    log(Info) << "[ASCRobotPd] controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ASCRobotPd::runController(atrias_msgs::robot_state rs) {
    // Set the PD gains
    // lLeg
    // MotorA
    P0.set(guiIn.p_gain);
    D0.set(guiIn.d_gain);
    // MotorB
    P1.set(guiIn.p_gain);
    D1.set(guiIn.d_gain);
    // Hip
    P2.set(guiIn.hip_p_gain);
    D2.set(guiIn.hip_d_gain);
    // rLeg
    // MotorA
    P3.set(guiIn.p_gain);
    D3.set(guiIn.d_gain);
    // MotorB
    P4.set(guiIn.p_gain);
    D4.set(guiIn.d_gain);
    // Hip
    P5.set(guiIn.hip_p_gain);
    D5.set(guiIn.hip_d_gain);

    // Transform from leg angle and length to motor positions
    leftMotorAngle = legToMotorPos(guiIn.leg_ang, guiIn.leg_len);
    rightMotorAngle = legToMotorPos(guiIn.leg_ang, guiIn.leg_len);

    // lLeg
    // Calculate motorA current
    targetPos = leftMotorAngle.A;
    currentPos = rs.lLeg.halfA.motorAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.halfA.motorVelocity;
    controllerOutput.lLeg.motorCurrentA = pd0Controller(targetPos, currentPos, targetVel, currentVel);

    // Calculate motorB current
    targetPos = leftMotorAngle.B;
    currentPos = rs.lLeg.halfB.motorAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.halfB.motorVelocity;
    controllerOutput.lLeg.motorCurrentB = pd1Controller(targetPos, currentPos, targetVel, currentVel);

    // Calculate hip current
    targetPos = guiIn.hip_ang;
    currentPos = rs.lLeg.hip.legBodyAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.hip.legBodyVelocity;
    controllerOutput.lLeg.motorCurrentHip = pd2Controller(targetPos, currentPos, targetVel, currentVel);

    // rLeg
    // Calculate motorA current
    targetPos = rightMotorAngle.A;
    currentPos = rs.rLeg.halfA.motorAngle;
    targetVel = 0.0;
    currentVel = rs.rLeg.halfA.motorVelocity;
    controllerOutput.rLeg.motorCurrentA = pd3Controller(targetPos, currentPos, targetVel, currentVel);

    // Calculate motorB current
    targetPos = rightMotorAngle.B;
    currentPos = rs.rLeg.halfB.motorAngle;
    targetVel = 0.0;
    currentVel = rs.rLeg.halfB.motorVelocity;
    controllerOutput.rLeg.motorCurrentB = pd4Controller(targetPos, currentPos, targetVel, currentVel);

    // Calculate hip current
    // TODO: Get correct values from the gui
    //targetPos = guiIn.hip_ang;
    currentPos = rs.rLeg.hip.legBodyAngle;
    targetVel = 0.0;
    currentVel = rs.rLeg.hip.legBodyVelocity;
    //controllerOutput.rLeg.motorCurrentHip = pd5Controller(targetPos, currentPos, targetVel, currentVel);

    controllerOutput.command = medulla_state_run;

    // If we're enabled, inform the GUI
    guiOut.isEnabled = (rs.cmState == (controllerManager::ControllerManagerState_t)controllerManager::ControllerManagerState::CONTROLLER_RUNNING);

    // Send data to the GUI
    if (pubTimer->readyToSend())
        guiDataOut.write(guiOut);

    return controllerOutput;
}

// Don't put control code below here!
bool ASCRobotPd::configureHook() {
    // Connect to the subcontrollers
    pd0 = this->getPeer(pd0Name);
    if (pd0)
        pd0Controller = pd0->provides("pd")->getOperation("runController");

    pd1 = this->getPeer(pd1Name);
    if (pd1)
        pd1Controller = pd1->provides("pd")->getOperation("runController");

    pd2 = this->getPeer(pd2Name);
    if (pd2)
        pd2Controller = pd2->provides("pd")->getOperation("runController");

    pd3 = this->getPeer(pd3Name);
    if (pd3)
        pd3Controller = pd3->provides("pd")->getOperation("runController");

    pd4 = this->getPeer(pd4Name);
    if (pd4)
        pd4Controller = pd4->provides("pd")->getOperation("runController");

    pd5 = this->getPeer(pd5Name);
    if (pd5)
        pd5Controller = pd5->provides("pd")->getOperation("runController");

    legToMotorPos = this->provides("legToMotorTransforms")->getOperation("posTransform");

    // Get references to the attributes
    P0 = pd0->properties()->getProperty("P");
    D0 = pd0->properties()->getProperty("D");
    P1 = pd1->properties()->getProperty("P");
    D1 = pd1->properties()->getProperty("D");
    P2 = pd2->properties()->getProperty("P");
    D2 = pd2->properties()->getProperty("D");
    P3 = pd3->properties()->getProperty("P");
    D3 = pd3->properties()->getProperty("D");
    P4 = pd4->properties()->getProperty("P");
    D4 = pd4->properties()->getProperty("D");
    P5 = pd5->properties()->getProperty("P");
    D5 = pd5->properties()->getProperty("D");

    log(Info) << "[ASCRobotPd] configured!" << endlog();
    return true;
}

bool ASCRobotPd::startHook() {
    log(Info) << "[ASCRobotPd] started!" << endlog();
    return true;
}

void ASCRobotPd::updateHook() {
    guiDataIn.read(guiIn);
}

void ASCRobotPd::stopHook() {
    log(Info) << "[ASCRobotPd] stopped!" << endlog();
}

void ASCRobotPd::cleanupHook() {
    delete pubTimer;
    log(Info) << "[ASCRobotPd] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCRobotPd)

}
}

