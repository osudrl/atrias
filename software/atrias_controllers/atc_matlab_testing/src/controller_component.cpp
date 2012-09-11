/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_matlab_testing controller.
 */

#include <atc_matlab_testing/controller_component.h>

namespace atrias {
namespace controller {

ATCMatlabTesting::ATCMatlabTesting(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in"),
    logPort(name + "_log")
{
    this->provides("atc")
        ->addOperation("runController", &ATCMatlabTesting::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

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

    log(Info) << "[ATCMT] atc_matlab_testing controller constructed!" << endlog();
}

atrias_msgs::controller_output ATCMatlabTesting::runController(atrias_msgs::robot_state rs) {
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

    // begin control code //

    // Inputs
    leg_position_pd_test_U.motorAnglesAB[0] = rs.lLeg.halfA.motorAngle;
    leg_position_pd_test_U.motorAnglesAB[1] = rs.lLeg.halfB.motorAngle;
    leg_position_pd_test_U.desiredAnglesLA_KA[0] = guiIn.leg_ang;
    leg_position_pd_test_U.desiredAnglesLA_KA[1] = guiIn.leg_len;

    // Setp the controller
    leg_position_pd_test_step();

    // Stuff the msg
    co.lLeg.motorCurrentA = leg_position_pd_test_Y.uA_uB[0];
    co.lLeg.motorCurrentB = leg_position_pd_test_Y.uA_uB[1];

    // end control code //

    // Command a run state
    co.command = medulla_state_run;

    // Stuff the msg and push to ROS for logging
    //logData.desiredState = 0.0;
    //logPort.write(logData);

    // Output for RTOps
    return co;
}

// Don't put control code below here!
bool ATCMatlabTesting::configureHook() {
    // Controller parameters
    leg_position_pd_test_P.Saturation_UpperSat = 40;
    leg_position_pd_test_P.Saturation_LowerSat = -40;
    leg_position_pd_test_P.LegAngleP_Gain = 600;
    leg_position_pd_test_P.LegAngleD_Gain = 20;
    leg_position_pd_test_P.KneeAngleP_Gain = 600;
    leg_position_pd_test_P.KneeAngleD_Gain = 20;

    // Initialize the controller
    leg_position_pd_test_initialize();
    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
}

bool ATCMatlabTesting::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCMatlabTesting::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCMatlabTesting::stopHook() {
    leg_position_pd_test_terminate();
    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCMatlabTesting::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCMatlabTesting)

}
}
