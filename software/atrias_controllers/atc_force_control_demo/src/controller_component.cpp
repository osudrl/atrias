/*! \file controller_component.cpp
 *  \author Mikhail Jones
 *  \brief Orocos Component code for the atc_force_control_demo controller.
 */

// Initialize ==================================================================
#include <atc_force_control_demo/controller_component.h>

double t = 0.0;

namespace atrias {
namespace controller {

// ATCForceControlDemo =========================================================
ATCForceControlDemo::ATCForceControlDemo(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log"),
    guiDataOut("gui_data_out"),
    guiDataIn("gui_data_in")
{
    this->provides("atc")->addOperation("runController", &ATCForceControlDemo::runController, this, ClientThread).doc("Get robot_state from RTOps and return controller output.");

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

// ATCForceControlDemo =========================================================
atrias_msgs::controller_output ATCForceControlDemo::runController(atrias_msgs::robot_state rs) {

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

    // BEGIN CONTROL CODE ******************************************************

	// Increment time step
	t = t + 0.001;

	// Leg position controller .................................................
	lMotorAngle = legToMotorPos(M_PI/2.0, 0.65);
	rMotorAngle = legToMotorPos(M_PI/2.0, 0.85);
	
	// Set motor currents
	co.lLeg.motorCurrentA = guiIn.leg_pos_p_gain*(lMotorAngle.A - rs.lLeg.halfA.motorAngle) + guiIn.leg_pos_d_gain*(0.0 - rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = guiIn.leg_pos_p_gain*(lMotorAngle.B - rs.lLeg.halfB.motorAngle) + guiIn.leg_pos_d_gain*(0.0 - rs.lLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentA = guiIn.leg_pos_p_gain*(rMotorAngle.A - rs.rLeg.halfA.motorAngle) + guiIn.leg_pos_d_gain*(0.0 - rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = guiIn.leg_pos_p_gain*(rMotorAngle.B - rs.rLeg.halfB.motorAngle) + guiIn.leg_pos_d_gain*(0.0 - rs.rLeg.halfB.motorVelocity);

	// Force controller ........................................................
	gain.kp = guiIn.leg_force_p_gain;
	gain.kd = guiIn.leg_force_d_gain;
	gain.ks = guiIn.robot_spring;
	gain.kg = guiIn.robot_gear;
	gain.kt = guiIn.robot_motor;
	
	if (guiIn.constant_force) {	
		// Get component forces from GUI, velocities equal zero.
		legForce.fx = guiIn.fx;
		legForce.fz = guiIn.fz;
		legForce.dfx = 0.0;
		legForce.dfz = 0.0;
		
		// Comute and set motor current values
		motorCurrent = legForceToMotorCurrent(legForce, gain, rs.rLeg, rs.position);
		co.rLeg.motorCurrentA = motorCurrent.A;
		co.rLeg.motorCurrentB = motorCurrent.B;
		
	} else if (guiIn.sinewave_force) {	
		// Compute sinewave forces
		// TODO - Generate smooth transitions between GUI changes.
		legForce.fx = guiIn.offsetx + guiIn.ampx*sin(t*2.0*M_PI*guiIn.freqx);
		legForce.fz = guiIn.offsetz + guiIn.ampz*sin(t*2.0*M_PI*guiIn.freqz);
		legForce.dfx = 2.0*M_PI*guiIn.ampx*guiIn.freqx*cos(t*2.0*M_PI*guiIn.freqx);
		legForce.dfz = 2.0*M_PI*guiIn.ampz*guiIn.freqz*cos(t*2.0*M_PI*guiIn.freqz);;
		
		// Comute and set motor current values
		motorCurrent = legForceToMotorCurrent(legForce, gain, rs.rLeg, rs.position);
		co.rLeg.motorCurrentA = motorCurrent.A;
		co.rLeg.motorCurrentB = motorCurrent.B;
		
	}
	
	// Hip controller ..........................................................
	if (guiIn.constant_hip) {		
		// Get hip angles from GUI
		hipAngle.left = guiIn.hip_angle;
		hipAngle.right = guiIn.hip_angle;
		
        // Set motor currents
        co.lLeg.motorCurrentHip = guiIn.hip_p_gain*(hipAngle.left - rs.lLeg.hip.legBodyAngle) + guiIn.hip_d_gain*(0.0 - rs.lLeg.hip.legBodyVelocity);
        co.rLeg.motorCurrentHip = guiIn.hip_p_gain*(hipAngle.right - rs.rLeg.hip.legBodyAngle) + guiIn.hip_d_gain*(0.0 - rs.rLeg.hip.legBodyVelocity);

	} else if (guiIn.advanced_hip) {
	
		// Get toe positions from GUI
		toePosition.left = guiIn.left_toe;
		toePosition.right = guiIn.right_toe;
		hipAngle = toePositionToHipAngle(toePosition, rs.lLeg, rs.rLeg, rs.position);
		
		// Set motor currents
        co.lLeg.motorCurrentHip = guiIn.hip_p_gain*(hipAngle.left - rs.lLeg.hip.legBodyAngle) + guiIn.hip_d_gain*(0.0 - rs.lLeg.hip.legBodyVelocity);
        co.rLeg.motorCurrentHip = guiIn.hip_p_gain*(hipAngle.right - rs.rLeg.hip.legBodyAngle) + guiIn.hip_d_gain*(0.0 - rs.rLeg.hip.legBodyVelocity);
	}


    // END CONTROL CODE ********************************************************

    // Command a run state
    co.command = medulla_state_run;

    // Let the GUI know the controller run state
    guiOut.isEnabled = ((rtOps::RtOpsState) rs.rtOpsState == rtOps::RtOpsState::ENABLED);

    // Send data to the GUI
    if (pubTimer->readyToSend()) guiDataOut.write(guiOut);

    // Stuff the msg and push to ROS for logging
    logData.desiredState = 0.0;
    logPort.write(logData);

    // Output for RTOps
    return co;

} // ATCForceControlDemo::runController

// DON'T PUT CONTROL CODE BELOW HERE! ******************************************

// configureHook ===============================================================
bool ATCForceControlDemo::configureHook() {

	// ASCLegToMotorTransforms Service
    legToMotorPos = this->provides("legToMotorTransforms")->getOperation("posTransform");

	// ASCLegForce Service
	legForceToMotorCurrent = this->provides("ascLegForce")->getOperation("legForceToMotorCurrent");

	// ASCHipInverseKinematics Service
	toePositionToHipAngle = this->provides("ascHipInverseKinematics")->getOperation("toePositionToHipAngle");

    log(Info) << "[ATCMT] configured!" << endlog();
    return true;

} // configureHook

// startHook ===================================================================
bool ATCForceControlDemo::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;

} // startHook

// updateHook ==================================================================
void ATCForceControlDemo::updateHook() {
    guiDataIn.read(guiIn);

} // updateHook

// stopHook ====================================================================
void ATCForceControlDemo::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();

} // stopHook

// cleanupHook =================================================================
void ATCForceControlDemo::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();

} // cleanupHook

ORO_CREATE_COMPONENT(ATCForceControlDemo)

} // namespace controller
} // namespace atrias
