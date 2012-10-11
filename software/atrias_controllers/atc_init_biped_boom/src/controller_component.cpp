/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the atc_init_biped_boom controller.
 */

#include <atc_init_biped_boom/controller_component.h>

namespace atrias {
namespace controller {

ATCInitBipedBoom::ATCInitBipedBoom(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log"),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCInitBipedBoom::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

    // Add properties
    this->addProperty("ASCInitBipedBoom0Name", ASCInitBipedBoom0Name);

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

    // Initial values
    cycle = 0;

    log(Info) << "[ATCMT] atc_init_biped_boom controller constructed!" << endlog();
}

atrias_msgs::controller_output ATCInitBipedBoom::runController(atrias_msgs::robot_state rs) {
    // Do nothing
    co.lLeg.motorCurrentA   = 0.0;
    co.lLeg.motorCurrentB   = 0.0;
    co.lLeg.motorCurrentHip = 0.0;
    co.rLeg.motorCurrentA   = 0.0;
    co.rLeg.motorCurrentB   = 0.0;
    co.rLeg.motorCurrentHip = 0.0;

    // Only run the controller when we're enabled
    if ((uint8_t)rs.cmState != (uint8_t)controllerManager::RtOpsCommand::ENABLE)
        return co;

    // Set the desired robot position
    if (cycle == 0)
    {
        pos.lLeg.A = M_PI/4;
        pos.lLeg.B = 3.0*M_PI/4;
        pos.lLeg.hip = 0.0;
        pos.rLeg.A = M_PI/4;
        pos.rLeg.B = 3.0*M_PI/4;
        pos.rLeg.hip = 0.0;
        initStatus = ASCInitBipedBoom0Init(rs, pos);
        if (initStatus == false)
            log(Error) << "[ATCMT] asc_init_biped_boom failed to initialize" << endlog();
        cycle++;
    }

    // Run the startup controller
    co = ASCInitBipedBoom0Run(rs);

    // Check to see if we are done
    // TODO: implement this in the subcontroller
    runStatus = ASCInitBipedBoom0Done();

    // Command a run state
    co.command = medulla_state_run;

    // Stuff the msg and push to ROS for logging
    logData.desiredState = 0.0;
    logPort.write(logData);

    // Output for RTOps
    return co;
}

// Don't put control code below here!
bool ATCInitBipedBoom::configureHook() {
    // Connect to the subcontrollers
    ASCInitBipedBoom0 = this->getPeer(ASCInitBipedBoom0Name);
    if (ASCInitBipedBoom0)
    {
        ASCInitBipedBoom0Init = ASCInitBipedBoom0->provides("ASCInitBipedBoom")->getOperation("init");
        ASCInitBipedBoom0Run = ASCInitBipedBoom0->provides("ASCInitBipedBoom")->getOperation("run");
        ASCInitBipedBoom0Done = ASCInitBipedBoom0->provides("ASCInitBipedBoom")->getOperation("done");
    }

    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
}

bool ATCInitBipedBoom::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCInitBipedBoom::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCInitBipedBoom::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCInitBipedBoom::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCInitBipedBoom)

}
}
