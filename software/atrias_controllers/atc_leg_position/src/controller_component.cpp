/*! \file controller_component.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Orocos Component code for the leg position controller.
 */

#include <atc_leg_position/controller_component.h>

namespace atrias {
namespace controller {

ATCLegPosition::ATCLegPosition(std::string name):
    RTT::TaskContext(name),
    guiDataIn("gui_data_in"),
    guiDataOut("gui_data_out")
{
    this->provides("atc")
        ->addOperation("runController", &ATCLegPosition::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add properties.
    this->addProperty("robotPd0Name", robotPd0Name);

    // Add ports.
    addEventPort(guiDataIn);
    addPort(guiDataOut);

    pubTimer = new GuiPublishTimer(20);

    // Defaults
    ds.left.leg.ang     = M_PI/2.0;
    ds.left.leg.angVel  = 0.0;
    ds.left.leg.len     = 0.75;
    ds.left.leg.lenVel  = 0.0;
    ds.left.hip.ang     = 3.0*M_PI/2.0;
    ds.left.hip.vel     = 0.0;
    ds.right.leg.ang    = M_PI/2.0;
    ds.right.leg.angVel = 0.0;
    ds.right.leg.len    = 0.75;
    ds.right.leg.lenVel = 0.0;
    ds.right.hip.ang    = 3.0*M_PI/2.0;
    ds.right.hip.vel    = 0.0;

    log(Info) << "[ATCLP] Leg position controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ATCLegPosition::runController(atrias_msgs::robot_state rs) {
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

    // Set the PD gains
    // Legs
    legP.set(guiIn.p_gain);
    legD.set(guiIn.d_gain);
    // Hips
    hipP.set(guiIn.hip_p_gain);
    hipD.set(guiIn.hip_d_gain);

    // Figure out the desired state
    ds.left.leg.ang     = guiIn.leg_ang;
    ds.left.leg.angVel  = 0.0;
    ds.left.leg.len     = guiIn.leg_len;
    ds.left.leg.lenVel  = 0.0;
    ds.left.hip.ang     = guiIn.hip_ang;
    ds.left.hip.vel     = 0.0;
    ds.right.leg.ang    = guiIn.leg_ang;
    ds.right.leg.angVel = 0.0;
    ds.right.leg.len    = guiIn.leg_len;
    ds.right.leg.lenVel = 0.0;
    ds.right.hip.ang    = 3.0*M_PI/2.0;
    ds.right.hip.vel    = 0.0;

    // Get the controller output
    co = robotPd0Controller(rs, ds);

    // If we're enabled, inform the GUI
    guiOut.isEnabled = (rs.cmState == (controllerManager::ControllerManagerState_t)controllerManager::ControllerManagerState::CONTROLLER_RUNNING);

    // Send data to the GUI
    if (pubTimer->readyToSend())
        guiDataOut.write(guiOut);

    return co;
}

// Don't put control code below here!
bool ATCLegPosition::configureHook() {
    // Connect to the subcontrollers
    robotPd0 = this->getPeer(robotPd0Name);
    if (robotPd0)
        robotPd0Controller = robotPd0->provides("robot_pd")->getOperation("runController");

    // Get references to the attributes
    legP = robotPd0->properties()->getProperty("legP");
    legD = robotPd0->properties()->getProperty("legD");
    hipP = robotPd0->properties()->getProperty("hipP");
    hipD = robotPd0->properties()->getProperty("hipD");

    log(Info) << "[ATCLP] configured!" << endlog();
    return true;
}

bool ATCLegPosition::startHook() {
    log(Info) << "[ATCLP] started!" << endlog();
    return true;
}

void ATCLegPosition::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCLegPosition::stopHook() {
    log(Info) << "[ATCLP] stopped!" << endlog();
}

void ATCLegPosition::cleanupHook() {
    delete pubTimer;
    log(Info) << "[ATCLP] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCLegPosition)

}
}

