/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_component controller.
 */

#include <atc_component/controller_component.h>

namespace atrias {
namespace controller {

ATCComponent::ATCComponent(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log"),
    guiDataOut("gui_data_out"),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCComponent::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add properties

    // For the GUI
    addEventPort(guiDataIn);
    addPort(guiDataOut);
    pubTimer = new GuiPublishTimer(20);

    // Logging
    // Create a port
    addPort(logPort);
    // Unbuffered
    ConnPolicy policy = RTT::ConnPolicy::data();
    // Transport type = ROS
    policy.transport = 3;
    // ROS topic name
    policy.name_id = "/" + name + "_log";
    // Construct the stream between the port and ROS topic
    logPort.createStream(policy);

    log(Info) << "[ATCMT] atc_component controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCComponent::runController(atrias_msgs::robot_state rs) {
    // Stuff the msg
    co.lLeg.motorCurrentA = guiIn.des_motor_torque_A;
    co.lLeg.motorCurrentB = guiIn.des_motor_torque_B;
    co.lLeg.motorCurrentHip = guiIn.des_motor_torque_hip;

    // Command a run state
    co.command = medulla_state_run;

    // Send data to the GUI
    if (pubTimer->readyToSend())
        guiDataOut.write(guiOut);

    // Stuff the msg and push to ROS for logging
    logData.desiredState = 0.0;
    logPort.write(logData);

    // Output for RTOps
    return co;
}

// Don't put control code below here!
bool ATCComponent::configureHook() {
    // Connect to the subcontrollers
    // Service plugins

    // Get references to subcontroller component properties

    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
}

bool ATCComponent::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCComponent::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCComponent::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCComponent::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCComponent)

}
}
