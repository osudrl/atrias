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
    simulink_name_U.x[0]  = rs.lLeg.hip.legBodyAngle;
    simulink_name_U.x[1]  = rs.lLeg.halfA.legAngle;
    simulink_name_U.x[2]  = rs.lLeg.halfA.motorAngle;
    simulink_name_U.x[3]  = rs.lLeg.halfB.legAngle;
    simulink_name_U.x[4]  = rs.lLeg.halfB.motorAngle;
    simulink_name_U.x[5]  = rs.rLeg.hip.legBodyAngle;
    simulink_name_U.x[6]  = rs.rLeg.halfA.legAngle;
    simulink_name_U.x[7]  = rs.rLeg.halfA.motorAngle;
    simulink_name_U.x[8]  = rs.rLeg.halfB.legAngle;
    simulink_name_U.x[9]  = rs.rLeg.halfB.motorAngle;
    simulink_name_U.x[10] = rs.position.xPosition;
    simulink_name_U.x[11] = rs.position.yPosition;
    simulink_name_U.x[12] = rs.position.zPosition;
    simulink_name_U.x[13] = rs.position.bodyPitch;

    // Set points
    simulink_name_U.set_point_input[0] = guiIn.q1r;
    simulink_name_U.set_point_input[1] = guiIn.q2r;
    simulink_name_U.set_point_input[2] = guiIn.q3r;
    simulink_name_U.set_point_input[3] = guiIn.q1l;
    simulink_name_U.set_point_input[4] = guiIn.q2l;
    simulink_name_U.set_point_input[5] = guiIn.q3l;

    // KP and KD
    simulink_name_U.kp[0] = guiIn.kp1;
    simulink_name_U.kp[1] = guiIn.kp2;
    simulink_name_U.kp[2] = guiIn.kp3;
    simulink_name_U.kd[0] = guiIn.kd1;
    simulink_name_U.kd[1] = guiIn.kd2;
    simulink_name_U.kd[2] = guiIn.kd3;

    // Epsilon
    simulink_name_U.epsilon[0] = guiIn.epsilon;


    // Step the controller
    simulink_name_step();


    // Stuff the msg
    co.lLeg.motorCurrentA   = simulink_name_Y.u[0];
    co.lLeg.motorCurrentB   = simulink_name_Y.u[1];
    co.lLeg.motorCurrentHip = simulink_name_Y.u[2];
    co.rLeg.motorCurrentA   = simulink_name_Y.u[3];
    co.rLeg.motorCurrentB   = simulink_name_Y.u[4];
    co.rLeg.motorCurrentHip = simulink_name_Y.u[5];

    // end control code //

    // Log the output
    logData.y1r = simulink_name_Y.y[0];
    logData.y2r = simulink_name_Y.y[1];
    logData.y3r = simulink_name_Y.y[2];
    logData.y1l = simulink_name_Y.y[3];
    logData.y2l = simulink_name_Y.y[4];
    logData.y3l = simulink_name_Y.y[5];

    logData.dy1r = simulink_name_Y.dy[0];
    logData.dy2r = simulink_name_Y.dy[1];
    logData.dy3r = simulink_name_Y.dy[2];
    logData.dy1l = simulink_name_Y.dy[3];
    logData.dy2l = simulink_name_Y.dy[4];
    logData.dy3l = simulink_name_Y.dy[5];

    logPort.write(logData);

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
