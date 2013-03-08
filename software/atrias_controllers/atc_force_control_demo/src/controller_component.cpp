/*! \file controller_component.cpp
 *  \author Mikhail Jones
 *  \brief Orocos Component code for the atc_force_control_demo controller.
 */

// Initialize ==================================================================
#include <atc_force_control_demo/controller_component.h>

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
    
    // ASCHipBoomKinematics
    this->addProperty("ascHipBoomKinematics0Name", ascHipBoomKinematics0Name);
    
	// ASCLegForceControl
	this->addProperty("ascLegForceControl0Name", ascLegForceControl0Name);

    // For the GUI
    addEventPort(guiDataIn);
    addPort(guiDataOut);
    pubTimer = new GuiPublishTimer(20);
    
    // Initalize variables
    t = 0.0;

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
	
    // Initialize variables
    legForce.fx = 0.0;
    legForce.fz = 0.0;
    legForce.dfx = 0.0;
    legForce.dfz = 0.0;
    gain.kp = guiIn.leg_for_kp;
	gain.kd = guiIn.leg_for_kd;
	gain.ks = guiIn.robot_ks;
	gain.kg = guiIn.robot_kg;
	gain.kt = guiIn.robot_kt;

	// Increment time step
	t = t + 0.001;


	// Left leg controller -----------------------------------------------------

	if (guiIn.left_leg_pos) {		
		// Set motor angles
		lMotorAngle = legToMotorPos(guiIn.left_leg_ang, guiIn.left_leg_len);

		// Set motor currents
		co.lLeg.motorCurrentA = guiIn.leg_pos_kp*(lMotorAngle.A - rs.lLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfA.motorVelocity);
		co.lLeg.motorCurrentB = guiIn.leg_pos_kp*(lMotorAngle.B - rs.lLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfB.motorVelocity);

	} else if (guiIn.left_leg_for) {
		// Get component forces from GUI, velocities equal to zero.
		legForce.fx = guiIn.left_fx;
		legForce.fz = guiIn.left_fz;
		legForce.dfx = 0.0;
		legForce.dfz = 0.0;
		
		// Comute and set motor current values
		motorCurrent = legForceToMotorCurrent0(legForce, gain, rs.lLeg, rs.position);
		co.lLeg.motorCurrentA = motorCurrent.A;
		co.lLeg.motorCurrentB = motorCurrent.B;
		
	} else if (guiIn.left_leg_wave_for) {
		// Compute sinewave forces
		legForce.fx = guiIn.left_offx + guiIn.left_ampx*sin(t*2.0*M_PI*guiIn.left_freqx);
		legForce.fz = guiIn.left_offz + guiIn.left_ampz*sin(t*2.0*M_PI*guiIn.left_freqz);
		legForce.dfx = 2.0*M_PI*guiIn.left_ampx*guiIn.left_freqx*cos(t*2.0*M_PI*guiIn.left_freqx);
		legForce.dfz = 2.0*M_PI*guiIn.left_ampz*guiIn.left_freqz*cos(t*2.0*M_PI*guiIn.left_freqz);;
		
		// Comute and set motor current values
		motorCurrent = legForceToMotorCurrent0(legForce, gain, rs.lLeg, rs.position);
		co.lLeg.motorCurrentA = motorCurrent.A;
		co.lLeg.motorCurrentB = motorCurrent.B;
		
	}


	// Right leg controller ----------------------------------------------------

	if (guiIn.right_leg_pos) {		
		// Set motor angles
		rMotorAngle = legToMotorPos(guiIn.right_leg_ang, guiIn.right_leg_len);

		// Set motor currents
		co.rLeg.motorCurrentA = guiIn.leg_pos_kp*(rMotorAngle.A - rs.rLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfA.motorVelocity);
		co.rLeg.motorCurrentB = guiIn.leg_pos_kp*(rMotorAngle.B - rs.rLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfB.motorVelocity);

	} else if (guiIn.right_leg_for) {
		// Get component forces from GUI, velocities equal to zero.
		legForce.fx = guiIn.right_fx;
		legForce.fz = guiIn.right_fz;
		legForce.dfx = 0.0;
		legForce.dfz = 0.0;
		
		// Comute and set motor current values
		motorCurrent = legForceToMotorCurrent0(legForce, gain, rs.rLeg, rs.position);
		co.rLeg.motorCurrentA = motorCurrent.A;
		co.rLeg.motorCurrentB = motorCurrent.B;
		
	} else if (guiIn.right_leg_wave_for) {
		// Compute sinewave forces
		legForce.fx = guiIn.right_offx + guiIn.right_ampx*sin(t*2.0*M_PI*guiIn.right_freqx);
		legForce.fz = guiIn.right_offz + guiIn.right_ampz*sin(t*2.0*M_PI*guiIn.right_freqz);
		legForce.dfx = 2.0*M_PI*guiIn.right_ampx*guiIn.right_freqx*cos(t*2.0*M_PI*guiIn.right_freqx);
		legForce.dfz = 2.0*M_PI*guiIn.right_ampz*guiIn.right_freqz*cos(t*2.0*M_PI*guiIn.right_freqz);;
		
		// Comute and set motor current values
		motorCurrent = legForceToMotorCurrent0(legForce, gain, rs.rLeg, rs.position);
		co.rLeg.motorCurrentA = motorCurrent.A;
		co.rLeg.motorCurrentB = motorCurrent.B;
		
	}

	
	// Hip controller ----------------------------------------------------------
	if (guiIn.constant_hip) {		
		// Get hip angles from GUI
		hipAngle.left = guiIn.left_hip_ang;
		hipAngle.right = guiIn.right_hip_ang;
		
        // Set motor currents
        co.lLeg.motorCurrentHip = guiIn.hip_pos_kp*(hipAngle.left - rs.lLeg.hip.legBodyAngle) + guiIn.hip_pos_kd*(0.0 - rs.lLeg.hip.legBodyVelocity);
        co.rLeg.motorCurrentHip = guiIn.hip_pos_kp*(hipAngle.right - rs.rLeg.hip.legBodyAngle) + guiIn.hip_pos_kd*(0.0 - rs.rLeg.hip.legBodyVelocity);

	} else if (guiIn.constant_toe) {	
		// Get toe positions from GUI
		toePosition.left = guiIn.left_toe_pos;
		toePosition.right = guiIn.right_toe_pos;
		
		// Compute inverse kinematics
		hipAngle = inverseKinematics0(toePosition, rs.lLeg, rs.rLeg, rs.position);
		
        // Set motor currents
        co.lLeg.motorCurrentHip = guiIn.hip_pos_kp*(hipAngle.left - rs.lLeg.hip.legBodyAngle) + guiIn.hip_pos_kd*(0.0 - rs.lLeg.hip.legBodyVelocity);
        co.rLeg.motorCurrentHip = guiIn.hip_pos_kp*(hipAngle.right - rs.rLeg.hip.legBodyAngle) + guiIn.hip_pos_kd*(0.0 - rs.rLeg.hip.legBodyVelocity);
        
	}


    // END CONTROL CODE ********************************************************

    // Command a run state
    co.command = medulla_state_run;

    // Let the GUI know the controller run state
    guiOut.isEnabled = ((rtOps::RtOpsState) rs.rtOpsState == rtOps::RtOpsState::ENABLED);

    // Send data to the GUI
    if (pubTimer->readyToSend()) guiDataOut.write(guiOut);

    // Output for RTOps
    return co;

} // ATCForceControlDemo::runController

// DON'T PUT CONTROL CODE BELOW HERE! ******************************************

// configureHook ===============================================================
bool ATCForceControlDemo::configureHook() {

	// ASCLegToMotorTransforms Service
    legToMotorPos = this->provides("legToMotorTransforms")->getOperation("posTransform");

	// ASCHipInverseKinematics Service
	ascHipBoomKinematics0 = this->getPeer(ascHipBoomKinematics0Name);
	if (ascHipBoomKinematics0) {
		inverseKinematics0 = ascHipBoomKinematics0->provides("ascHipBoomKinematics")->getOperation("inverseKinematics");
	}
	
	// ASCLegForceControl Service
	ascLegForceControl0 = this->getPeer(ascLegForceControl0Name);
	if (ascLegForceControl0) {
		legForceToMotorCurrent0 = ascLegForceControl0->provides("ascLegForceControl")->getOperation("legForceToMotorCurrent");
	}
      
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
