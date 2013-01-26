/*! \file controller_component.cpp
 *  \author Daniel Renjewski 01/25/2013
 *  \brief Orocos Component code for the atc_joint_position controller.
 */

#include <atc_joint_position/controller_component.h>

namespace atrias {
namespace controller {

ATCJointPosition::ATCJointPosition(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log"),
    guiDataOut("gui_data_out"),
    guiDataIn("gui_data_in")
{
    this->provides("atc")
        ->addOperation("runController", &ATCJointPosition::runController, this, ClientThread)
        .doc("Get robot_state from RTOps and return controller output.");

	 // Add properties.
    this->addProperty("pd0Name", pd0Name)
        .doc("Name of 0th PD subcontroller.");
    this->addProperty("pd1Name", pd1Name)
        .doc("Name of 1st PD subcontroller.");
    this->addProperty("pd2Name", pd2Name)
        .doc("Name of 2th PD subcontroller.");
    this->addProperty("pd3Name", pd3Name)
        .doc("Name of 3st PD subcontroller.");
    this->addProperty("pd4Name", pd4Name)
        .doc("Name of 4st PD subcontroller.");
    this->addProperty("pd5Name", pd5Name)
        .doc("Name of 5st PD subcontroller.");

    // For the GUI
    addEventPort(guiDataIn);
    addPort(guiDataOut);
    pubTimer = new GuiPublishTimer(20);

    // Logging
    // Create a port
    addPort(logPort);
    // Buffer port so we capture all data.
    ConnPolicy policy = RTT::ConnPolicy::buffer(100000);
    // Transport type = ROS
    policy.transport = 3;
    // ROS topic name
    policy.name_id = "/" + name + "_log";
    // Construct the stream between the port and ROS topic
    logPort.createStream(policy);

    log(Info) << "[ATCMT] Constructed!" << endlog();
}

atrias_msgs::controller_output ATCJointPosition::runController(atrias_msgs::robot_state rs) {
    // Do nothing unless told otherwise
    co.lLeg.motorCurrentA   = 0.0;
    co.lLeg.motorCurrentB   = 0.0;
    co.lLeg.motorCurrentHip = 0.0;
    co.rLeg.motorCurrentA   = 0.0;
    co.rLeg.motorCurrentB   = 0.0;
    co.rLeg.motorCurrentHip = 0.0;

    // Only run the controller when we're enabled
    if ((rtOps::RtOpsState)rs.rtOpsState != rtOps::RtOpsState::ENABLED)
        return co;

    // begin control code //


	//++++++++++++++++++++++++++++++++++++++++++++++++++leg++++++++++++++++++++++++++++++++++++++++++++++
	if (guiIn.sync)
	{
		leftMotorAngle = legToMotorPos(guiIn.a_ll,guiIn.l_ll);
		rightMotorAngle = leftMotorAngle;
	} else
	{
		leftMotorAngle = legToMotorPos(guiIn.a_ll,guiIn.l_ll);
		rightMotorAngle = legToMotorPos(guiIn.a_rl,guiIn.l_rl);
	}
	D0.set(guiIn.d_ll);
	P0.set(guiIn.p_ll);
	D1.set(guiIn.d_ll);
	P1.set(guiIn.p_ll);
	D3.set(guiIn.d_lr);
	P3.set(guiIn.p_lr);
	D4.set(guiIn.d_lr);
	P4.set(guiIn.p_lr);
	co.lLeg.motorCurrentA=pd0Controller(leftMotorAngle.A,rs.lLeg.halfA.motorAngle,0,rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB=pd1Controller(leftMotorAngle.B,rs.lLeg.halfB.motorAngle,0,rs.lLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentA=pd3Controller(rightMotorAngle.A,rs.rLeg.halfA.motorAngle,0,rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB=pd4Controller(rightMotorAngle.B,rs.rLeg.halfB.motorAngle,0,rs.rLeg.halfB.motorVelocity);

	//++++++++++++++++++++++++++++++++++++++++++++++++++hip++++++++++++++++++++++++++++++++++++++++++++++
	if (guiIn.vert)
	{
		hipangle_l=0;
		hipangle_r=0;
	} else
	{
		hipangle_l=guiIn.a_hl;
		hipangle_r=guiIn.a_hr;
	}
	D2.set(guiIn.d_hl);
	P2.set(guiIn.p_hl);
	D5.set(guiIn.d_hr);
	P5.set(guiIn.p_hr);
	co.lLeg.motorCurrentHip = pd2Controller(hipangle_l, rs.lLeg.hip.legBodyAngle, 0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentHip = pd5Controller(hipangle_r, rs.rLeg.hip.legBodyAngle, 0, rs.rLeg.hip.legBodyVelocity);

    // end control code //

    // Command a run state
    co.command = medulla_state_run;

    // Let the GUI know the controller run state
    guiOut.isEnabled = ((rtOps::RtOpsState) rs.rtOpsState == rtOps::RtOpsState::ENABLED);
    // Send data to the GUI
    if (pubTimer->readyToSend())
        guiDataOut.write(guiOut);

    // Stuff the msg and push to ROS for logging
    logData.desiredState = 0.0;
    logPort.write(logData);

    // Output for RTOps
    return co;
}

// Don't put control code below here!
bool ATCJointPosition::configureHook() {
    log(Info) << "[ATCMT] configured!" << endlog();
	legToMotorPos = this->provides("legToMotorTransforms")->getOperation("posTransform");
    return true;
}

bool ATCJointPosition::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
}

void ATCJointPosition::updateHook() {
    guiDataIn.read(guiIn);
}

void ATCJointPosition::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
}

void ATCJointPosition::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ATCJointPosition)

}
}
