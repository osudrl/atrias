/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_umich_2 controller.
 */

#include <atc_umich_2/controller_component.h>

namespace atrias {
namespace controller {

ATCUmich2::ATCUmich2(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in"),
    guiDataOut("gui_data_out"),
    logPort("logOutput")
{
    this->provides("atc")
        ->addOperation("runController", &ATCUmich2::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // For the GUI
    addEventPort(guiDataIn);
    addPort(guiDataOut);

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

atrias_msgs::controller_output ATCUmich2::runController(atrias_msgs::robot_state rs) {
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
    vc_controller_3_U.q_osu[0]  = rs.lLeg.hip.legBodyAngle;
    vc_controller_3_U.q_osu[1]  = rs.lLeg.halfA.legAngle;
    vc_controller_3_U.q_osu[2]  = rs.lLeg.halfA.motorAngle;
    vc_controller_3_U.q_osu[3]  = rs.lLeg.halfB.legAngle;
    vc_controller_3_U.q_osu[4]  = rs.lLeg.halfB.motorAngle;
    vc_controller_3_U.q_osu[5]  = rs.rLeg.hip.legBodyAngle;
    vc_controller_3_U.q_osu[6]  = rs.rLeg.halfA.legAngle;
    vc_controller_3_U.q_osu[7]  = rs.rLeg.halfA.motorAngle;
    vc_controller_3_U.q_osu[8]  = rs.rLeg.halfB.legAngle;
    vc_controller_3_U.q_osu[9]  = rs.rLeg.halfB.motorAngle;
    vc_controller_3_U.q_osu[10] = rs.position.xPosition;
    vc_controller_3_U.q_osu[11] = rs.position.yPosition;
    vc_controller_3_U.q_osu[12] = rs.position.zPosition;
    vc_controller_3_U.q_osu[13] = rs.position.bodyPitch;

    // KP and KD
    vc_controller_3_U.kp[0] = guiIn.kp1;
    vc_controller_3_U.kp[1] = guiIn.kp2;
    vc_controller_3_U.kp[2] = guiIn.kp3;
    vc_controller_3_U.kd[0] = guiIn.kd1;
    vc_controller_3_U.kd[1] = guiIn.kd2;
    vc_controller_3_U.kd[2] = guiIn.kd3;

    // Epsilon
    vc_controller_3_U.epsilon = guiIn.epsilon;

    // Cap value
    vc_controller_3_U.sat_val[0] = guiIn.leg_saturation_cap;
    vc_controller_3_U.sat_val[1] = guiIn.hip_saturation_cap;

    // Torso offset
    vc_controller_3_U.torso_offset = guiIn.torso_offset;

    // What type of "walking" are we doing?
    vc_controller_3_U.s_mode = guiIn.s_mode;

    // Walking frequency
    vc_controller_3_U.s_freq = guiIn.s_freq;

    // Stance leg
    vc_controller_3_U.stance_leg = guiIn.stance_leg;

    // Set the hip angles
    vc_controller_3_U.q3_des[0] = guiIn.q3_des[0];
    vc_controller_3_U.q3_des[1] = guiIn.q3_des[1];

    // What type of swapping are we doing?
    vc_controller_3_U.swap = guiIn.swap;

    // What type of thresholding are doing?
    vc_controller_3_U.swap_threshold[0] = guiIn.swap_threshold[0];
    vc_controller_3_U.swap_threshold[1] = guiIn.swap_threshold[1];
    vc_controller_3_U.swap_threshold[2] = guiIn.swap_threshold[2];

    // Pass the scuffing parameters
    vc_controller_3_U.scuff[0] = guiIn.scuff[0];
    vc_controller_3_U.scuff[1] = guiIn.scuff[1];


    // Step the controller
    vc_controller_3_step();


    // Stuff the msg
    co.lLeg.motorCurrentA   = vc_controller_3_Y.u[0];
    co.lLeg.motorCurrentB   = vc_controller_3_Y.u[1];
    co.lLeg.motorCurrentHip = vc_controller_3_Y.u[2];
    co.rLeg.motorCurrentA   = vc_controller_3_Y.u[3];
    co.rLeg.motorCurrentB   = vc_controller_3_Y.u[4];
    co.rLeg.motorCurrentHip = vc_controller_3_Y.u[5];

    // end control code //

    // Output info to the gui
    if (pubTimer->readyToSend())
    {
        guiOut.yr[0] = vc_controller_3_Y.y[0];
        guiOut.yr[1] = vc_controller_3_Y.y[1];
        guiOut.yr[2] = vc_controller_3_Y.y[2];
        guiOut.yl[0] = vc_controller_3_Y.y[3];
        guiOut.yl[1] = vc_controller_3_Y.y[4];
        guiOut.yl[2] = vc_controller_3_Y.y[5];

        guiOut.dyr[0] = vc_controller_3_Y.dy[0];
        guiOut.dyr[1] = vc_controller_3_Y.dy[1];
        guiOut.dyr[2] = vc_controller_3_Y.dy[2];
        guiOut.dyl[0] = vc_controller_3_Y.dy[3];
        guiOut.dyl[1] = vc_controller_3_Y.dy[4];
        guiOut.dyl[2] = vc_controller_3_Y.dy[5];

        guiOut.s = vc_controller_3_Y.s;
        guiOut.ds = vc_controller_3_Y.ds;

        guiDataOut.write(guiOut);
    }

    // Log the output
    logData.y1r = vc_controller_3_Y.y[0];
    logData.y2r = vc_controller_3_Y.y[1];
    logData.y3r = vc_controller_3_Y.y[2];
    logData.y1l = vc_controller_3_Y.y[3];
    logData.y2l = vc_controller_3_Y.y[4];
    logData.y3l = vc_controller_3_Y.y[5];

    logData.dy1r = vc_controller_3_Y.dy[0];
    logData.dy2r = vc_controller_3_Y.dy[1];
    logData.dy3r = vc_controller_3_Y.dy[2];
    logData.dy1l = vc_controller_3_Y.dy[3];
    logData.dy2l = vc_controller_3_Y.dy[4];
    logData.dy3l = vc_controller_3_Y.dy[5];

    logData.s  = vc_controller_3_Y.s;
    logData.ds = vc_controller_3_Y.ds;

    logPort.write(logData);

    // Command a run state
    co.command = medulla_state_run;

    // Output for RTOps
    return co;
}

// Don't put control code below here!
bool ATCUmich2::configureHook() {
    // Initialize the controller
    vc_controller_3_initialize();

    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
}

bool ATCUmich2::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCUmich2::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCUmich2::stopHook() {
    // Stop the controller
    vc_controller_3_terminate();

    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCUmich2::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCUmich2)

}
}
