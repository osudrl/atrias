/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_init_biped_boom controller.
 */

#include <atc_init_biped_boom/controller_component.h>

namespace atrias {
namespace controller {

ATCInitBipedBoomTest::ATCInitBipedBoomTest(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log"),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCInitBipedBoomTest::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add properties
    this->addProperty("initBipedBoom0Name", initBipedBoom0Name);

    // For the GUI
    addEventPort(guiDataIn);
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

    log(Info) << "[ATCMT] atc_init_biped_boom controller constructed!" << endlog();
}

atrias_msgs::controller_output ATCInitBipedBoomTest::runController(atrias_msgs::robot_state rs) {
    // Only run the controller when we're enabled
    if ((uint8_t)rs.cmState != (uint8_t)controllerManager::RtOpsCommand::ENABLE)
    {
        // Do nothing
        co.lLeg.motorCurrentA = 0.0;
        co.lLeg.motorCurrentB = 0.0;
        co.lLeg.motorCurrentHip = 0.0;
        co.rLeg.motorCurrentA = 0.0;
        co.rLeg.motorCurrentB = 0.0;
        co.rLeg.motorCurrentHip = 0.0;
        return co;
    }

    // Command a run state
    co.command = medulla_state_run;

    // Stuff the msg and push to ROS for logging
    logData.desiredState = 0.0;
    logPort.write(logData);

    // Output for RTOps
    return co;
}

// Don't put control code below here!
bool ATCInitBipedBoomTest::configureHook() {
    // Connect to the subcontrollers
    initBipedBoom0 = this->getPeer(initBipedBoom0Name);
    if (initBipedBoom0)
        initBipedBoom0Done = initBipedBoom0->provides("initBipedBoom")->getOperation("done");

    initBipedBoom0 = this->getPeer(initBipedBoom0Name);
    if (initBipedBoom0)
        initBipedBoom0Run = initBipedBoom0->provides("initBipedBoom")->getOperation("run");

    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
}

bool ATCInitBipedBoomTest::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCInitBipedBoomTest::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCInitBipedBoomTest::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCInitBipedBoomTest::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCInitBipedBoomTest)

}
}
