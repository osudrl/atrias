/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_umich_1 controller.
 */

#include <atc_umich_1/controller_component.h>

namespace atrias {
namespace controller {

ATCUmich1::ATCUmich1(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in"),
    logPort("logOutput")
{
    this->provides("atc")
        ->addOperation("runController", &ATCUmich1::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // For the GUI
    addEventPort(guiDataIn);
    pubTimer = new GuiPublishTimer(20);

    // For logging
    // Create a port
    addPort(logPort); 
    // Connect with buffer size 100000 so we get all data.
    ConnPolicy policy = RTT::ConnPolicy::buffer(100000);
    // Transport type = ROS
    policy.transport = 3;
    // ROS topic name
    policy.name_id = "/" + name + "_log";
    // Construct the stream between the port and ROS topic
    logPort.createStream(policy);


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
    // Robot state
    posing_controller_v2_U.q_osu[0]  = rs.lLeg.hip.legBodyAngle;
    posing_controller_v2_U.q_osu[1]  = rs.lLeg.halfA.legAngle;
    posing_controller_v2_U.q_osu[2]  = rs.lLeg.halfA.motorAngle;
    posing_controller_v2_U.q_osu[3]  = rs.lLeg.halfB.legAngle;
    posing_controller_v2_U.q_osu[4]  = rs.lLeg.halfB.motorAngle;
    posing_controller_v2_U.q_osu[5]  = rs.rLeg.hip.legBodyAngle;
    posing_controller_v2_U.q_osu[6]  = rs.rLeg.halfA.legAngle;
    posing_controller_v2_U.q_osu[7]  = rs.rLeg.halfA.motorAngle;
    posing_controller_v2_U.q_osu[8]  = rs.rLeg.halfB.legAngle;
    posing_controller_v2_U.q_osu[9]  = rs.rLeg.halfB.motorAngle;
    posing_controller_v2_U.q_osu[10] = rs.position.xPosition;
    posing_controller_v2_U.q_osu[11] = rs.position.yPosition;
    posing_controller_v2_U.q_osu[12] = rs.position.zPosition;
    posing_controller_v2_U.q_osu[13] = rs.position.bodyPitch;

    // Set points
    posing_controller_v2_U.set_point[0] = guiIn.q1r;
    posing_controller_v2_U.set_point[1] = guiIn.q2r;
    posing_controller_v2_U.set_point[2] = guiIn.q3r;
    posing_controller_v2_U.set_point[3] = guiIn.q1l;
    posing_controller_v2_U.set_point[4] = guiIn.q2l;
    posing_controller_v2_U.set_point[5] = guiIn.q3l;

    // KP and KD
    posing_controller_v2_U.kp[0] = guiIn.kp1;
    posing_controller_v2_U.kp[1] = guiIn.kp2;
    posing_controller_v2_U.kp[2] = guiIn.kp3;
    posing_controller_v2_U.kd[0] = guiIn.kd1;
    posing_controller_v2_U.kd[1] = guiIn.kd2;
    posing_controller_v2_U.kd[2] = guiIn.kd3;

    // Epsilon
    posing_controller_v2_U.epsilon = guiIn.epsilon;


    // Step the controller
    posing_controller_v2_step();


    // Stuff the msg
    co.lLeg.motorCurrentA   = posing_controller_v2_Y.u[0];
    co.lLeg.motorCurrentB   = posing_controller_v2_Y.u[1];
    co.lLeg.motorCurrentHip = posing_controller_v2_Y.u[2];
    co.rLeg.motorCurrentA   = posing_controller_v2_Y.u[3];
    co.rLeg.motorCurrentB   = posing_controller_v2_Y.u[4];
    co.rLeg.motorCurrentHip = posing_controller_v2_Y.u[5];

    // end control code //

    // Log the output
    logData.y1r = posing_controller_v2_Y.y[0];
    logData.y2r = posing_controller_v2_Y.y[1];
    logData.y3r = posing_controller_v2_Y.y[2];
    logData.y1l = posing_controller_v2_Y.y[3];
    logData.y2l = posing_controller_v2_Y.y[4];
    logData.y3l = posing_controller_v2_Y.y[5];

    logData.dy1r = posing_controller_v2_Y.dy[0];
    logData.dy2r = posing_controller_v2_Y.dy[1];
    logData.dy3r = posing_controller_v2_Y.dy[2];
    logData.dy1l = posing_controller_v2_Y.dy[3];
    logData.dy2l = posing_controller_v2_Y.dy[4];
    logData.dy3l = posing_controller_v2_Y.dy[5];

    logPort.write(logData);

    // Command a run state
    co.command = medulla_state_run;

    // Output for RTOps
    return co;
}

// Don't put control code below here!
bool ATCUmich1::configureHook() {
    // Initialize the controller
    posing_controller_v2_initialize();

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
    posing_controller_v2_terminate();

    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCUmich1::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCUmich1)

}
}
