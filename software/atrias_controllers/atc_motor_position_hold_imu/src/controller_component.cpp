/*! \file controller_component.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Orocos Component code for the motor position controller.
 */

#include <atc_motor_position_hold_imu/controller_component.h>

namespace atrias {
namespace controller {

ATCMotorPosition::ATCMotorPosition(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in"),
    guiDataOut("gui_data_out")
{
    this->provides("atc")
        ->addOperation("runController", &ATCMotorPosition::runController, this, ClientThread)
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

    log(Info) << "[ATCMP] Motor position controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCMotorPosition::runController(atrias_msgs::robot_state rs) {
    // Set the PD gains
    // lLeg
    // MotorA
    P0.set(guiIn.leg_motor_p_gain);
    D0.set(guiIn.leg_motor_d_gain);
    // MotorB
    P1.set(guiIn.leg_motor_p_gain);
    D1.set(guiIn.leg_motor_d_gain);
    // Hip
    P2.set(guiIn.hip_motor_p_gain);
    D2.set(guiIn.hip_motor_d_gain);

    // rLeg
    // MotorA
    P3.set(guiIn.leg_motor_p_gain);
    D3.set(guiIn.leg_motor_d_gain);
    // MotorB
    P4.set(guiIn.leg_motor_p_gain);
    D4.set(guiIn.leg_motor_d_gain);
    // Hip
    P5.set(guiIn.hip_motor_p_gain);
    D5.set(guiIn.hip_motor_d_gain);

    // Try to point the legs down, the right leg slightly in front of the left.
    static const double OFFSET = M_PI/18;
    targetLA = rs.position.imuPitch + M_PI/12 - OFFSET;
    targetLB = rs.position.imuPitch - M_PI/12 - OFFSET;
    targetRA = rs.position.imuPitch + M_PI/12 + OFFSET;
    targetRB = rs.position.imuPitch - M_PI/12 + OFFSET;

    // lLeg
    // motorA
    targetPos = targetLA;
    currentPos = rs.lLeg.halfA.motorAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.halfA.motorVelocity;
    co.lLeg.motorCurrentA = pd0Controller(targetPos, currentPos, targetVel, currentVel);

    // motorB
    targetPos = targetLB;
    currentPos = rs.lLeg.halfB.motorAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.halfB.motorVelocity;
    co.lLeg.motorCurrentB = pd1Controller(targetPos, currentPos, targetVel, currentVel);

    // Hip
    targetPos = guiIn.des_motor_ang_left_hip;
    currentPos = rs.lLeg.hip.legBodyAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.hip.legBodyVelocity;
    co.lLeg.motorCurrentHip = pd2Controller(targetPos, currentPos, targetVel, currentVel);

    // rLeg
    // motorA
    targetPos = targetRA;
    currentPos = rs.rLeg.halfA.motorAngle;
    targetVel = 0.0;
    currentVel = rs.rLeg.halfA.motorVelocity;
    co.rLeg.motorCurrentA = pd3Controller(targetPos, currentPos, targetVel, currentVel);

    // motorB
    targetPos = targetRB;
    currentPos = rs.rLeg.halfB.motorAngle;
    targetVel = 0.0;
    currentVel = rs.rLeg.halfB.motorVelocity;
    co.rLeg.motorCurrentB = pd4Controller(targetPos, currentPos, targetVel, currentVel);

    // Hip
    targetPos = guiIn.des_motor_ang_right_hip;
    currentPos = rs.rLeg.hip.legBodyAngle;
    targetVel = 0.0;
    currentVel = rs.rLeg.hip.legBodyVelocity;
    co.rLeg.motorCurrentHip = pd5Controller(targetPos, currentPos, targetVel, currentVel);

    co.command = medulla_state_run;

    // If we're enabled, inform the GUI
    guiOut.isEnabled = ((rtOps::RtOpsState) rs.rtOpsState == rtOps::RtOpsState::ENABLED);

    // Send data to the GUI
    if (pubTimer->readyToSend())
        guiDataOut.write(guiOut);

    return co;
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

// vim: expandtab:sts=4
