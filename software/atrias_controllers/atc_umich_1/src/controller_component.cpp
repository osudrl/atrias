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

    // Inputs
    simulink_name_U.simulink_robot_input[0]  = rs.lLeg.hip.legBodyAngle;
    simulink_name_U.simulink_robot_input[1]  = rs.lLeg.halfA.legAngle;
    simulink_name_U.simulink_robot_input[2]  = rs.lLeg.halfA.motorAngle;
    simulink_name_U.simulink_robot_input[3]  = rs.lLeg.halfB.legAngle;
    simulink_name_U.simulink_robot_input[4]  = rs.lLeg.halfB.motorAngle;
    simulink_name_U.simulink_robot_input[5]  = rs.rLeg.hip.legBodyAngle;
    simulink_name_U.simulink_robot_input[6]  = rs.rLeg.halfA.legAngle;
    simulink_name_U.simulink_robot_input[7]  = rs.rLeg.halfA.motorAngle;
    simulink_name_U.simulink_robot_input[8]  = rs.rLeg.halfB.legAngle;
    simulink_name_U.simulink_robot_input[9]  = rs.rLeg.halfB.motorAngle;
    simulink_name_U.simulink_robot_input[10] = rs.position.xPosition;
    simulink_name_U.simulink_robot_input[11] = rs.position.yPosition;
    simulink_name_U.simulink_robot_input[12] = rs.position.zPosition;
    simulink_name_U.simulink_robot_input[13] = rs.position.bodyPitch;

    simulink_name_U.simulink_set_point_input[0] = guiIn.q1r;
    simulink_name_U.simulink_set_point_input[1] = guiIn.q2r;
    simulink_name_U.simulink_set_point_input[2] = guiIn.q3r;
    simulink_name_U.simulink_set_point_input[3] = guiIn.q1l;
    simulink_name_U.simulink_set_point_input[4] = guiIn.q2l;
    simulink_name_U.simulink_set_point_input[5] = guiIn.q3l;

    // Step the controller
    simulink_name_step();

    // Stuff the msg
    co.lLeg.motorCurrentA   = simulink_name_Y.simulink_output[0];
    co.lLeg.motorCurrentB   = simulink_name_Y.simulink_output[1];
    co.lLeg.motorCurrentHip = simulink_name_Y.simulink_output[2];
    co.rLeg.motorCurrentA   = simulink_name_Y.simulink_output[3];
    co.rLeg.motorCurrentB   = simulink_name_Y.simulink_output[4];
    co.rLeg.motorCurrentHip = simulink_name_Y.simulink_output[5];

    // end control code //

    // Command a run state
    co.command = medulla_state_run;

    // Output for RTOps
    return co;
}

// Don't put control code below here!
bool ATCUmich1::configureHook() {
    // Initialize the controller
    simulink_name_initialize();

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
    // Stop the controller
    simulink_name_terminate();

    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCUmich1::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCUmich1)

}
}
