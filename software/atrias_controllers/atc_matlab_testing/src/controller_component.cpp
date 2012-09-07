/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_matlab_testing controller.
 */

#include <atc_matlab_testing/controller_component.h>

namespace atrias {
namespace controller {

ATCMatlabTesting::ATCMatlabTesting(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log"),
    guiDataIn("gui_data_in")
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
    // TODO: Fill these out
    leg_position_pd_test_U.motorAnglesAB[0] = ;
    leg_position_pd_test_U.motorAnglesAB[1] = ;
    leg_position_pd_test_U.desiredAnglesLA_KA[0] = ;
    leg_position_pd_test_U.desiredAnglesLA_KA[1] = ;

    // Setp the controller
    leg_position_pd_test_step();
    //printf("Out1 = %f\n", leg_position_pd_test_Y.Out1);
    //printf("Out2 = %f\n", leg_position_pd_test_Y.Out2);

    // Stuff the msg
    co.lLeg.motorCurrentA = leg_position_pd_test_Y.uA_uB[0];
    co.lLeg.motorCurrentB = leg_position_pd_test_Y.uA_uB[1];

    //co.lLeg.motorCurrentA = guiIn.des_motor_torque_A;
    //co.lLeg.motorCurrentB = guiIn.des_motor_torque_B;
    //co.lLeg.motorCurrentHip = guiIn.des_motor_torque_hip;

    // end control code //

    // Command a run state
    co.command = medulla_state_run;

    // Stuff the msg and push to ROS for logging
    logData.desiredState = 0.0;
    logPort.write(logData);

    // Output for RTOps
    return co;
}

// Don't put control code below here!
bool ATCMatlabTesting::configureHook() {
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
