/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_umich_1 controller.
 */

#include <atc_umich_1/controller_component.h>

namespace atrias {
namespace controller {

ATCUmich1::ATCUmich1(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCUmich1::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // For the GUI
    addEventPort(guiDataIn);
    pubTimer = new GuiPublishTimer(20);

    log(Info) << "[ATCMT] Constructed!" << endlog();
}

atrias_msgs::controller_output ATCUmich1::runController(atrias_msgs::robot_state rs) {
    // Do nothing unless told otherwise
    co.lLeg.motorCurrentA   = 0.0;
    co.lLeg.motorCurrentB   = 0.0;
    co.lLeg.motorCurrentHip = 0.0;
    co.rLeg.motorCurrentA   = 0.0;
    co.rLeg.motorCurrentB   = 0.0;
    co.rLeg.motorCurrentHip = 0.0;

    // Only run the controller when we're enabled
    if ((uint8_t)rs.cmState != (uint8_t)controllerManager::RtOpsCommand::ENABLE)
        return co;

    // begin control code //

    // Stuff the msg
    //co.rLeg.motorCurrentA   = guiIn.q1r;
    //co.rLeg.motorCurrentB   = guiIn.q2r;
    //co.rLeg.motorCurrentHip = guiIn.q3r;
    //co.lLeg.motorCurrentA   = guiIn.q1l;
    //co.lLeg.motorCurrentB   = guiIn.q2l;
    //co.lLeg.motorCurrentHip = guiIn.q3l;

    // end control code //

    // Command a run state
    co.command = medulla_state_run;

    // Output for RTOps
    return co;
}

// Don't put control code below here!
bool ATCUmich1::configureHook() {
    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
}

bool ATCUmich1::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCUmich1::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCUmich1::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCUmich1::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCUmich1)

}
}
