/*! \file controller_component.cpp
 *  \author Mikhail Jones
 *  \brief Orocos Component code for the atc_vertical_force_control_hopping controller.
 */

// Initialize ==================================================================
#include <atc_vertical_force_control_hopping/controller_component.h>

namespace atrias {
namespace controller {

// ATCVerticalForceControlHopping =========================================================
ATCVerticalForceControlHopping::ATCVerticalForceControlHopping(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log"),
    guiDataOut("gui_data_out"),
    guiDataIn("gui_data_in")
{
    this->provides("atc")->addOperation("runController", &ATCVerticalForceControlHopping::runController, this, ClientThread).doc("Get robot_state from RTOps and return controller output.");
    
    // ASCHipBoomKinematics
    this->addProperty("ascHipBoomKinematics0Name", ascHipBoomKinematics0Name);
    
	// ASCLegForceControl
	this->addProperty("ascLegForceControl0Name", ascLegForceControl0Name);

	// ASCSlipModel
	this->addProperty("ascSlipModel0Name", ascSlipModel0Name);

    // For the GUI
    addEventPort(guiDataIn);
    addPort(guiDataOut);
    pubTimer = new GuiPublishTimer(20);
    
    // Initalize variables
    isStance = false;
    isLeftStance = false;
    isRightStance = false;

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

// ATCVerticalForceControlHopping ==============================================
atrias_msgs::controller_output ATCVerticalForceControlHopping::runController(atrias_msgs::robot_state rs) {

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

	// Set force control gains
	gain.kp = guiIn.leg_for_kp;
	gain.kd = guiIn.leg_for_kd;
	gain.ks = guiIn.robot_ks;
	gain.kg = guiIn.robot_kg;
	gain.kt = guiIn.robot_kt;
	
	// Robot parameters.
	slipModel.g = -9.81;
	slipModel.m = guiIn.slip_m;
	slipModel.r0 = guiIn.slip_l0;
	
	// Decide which are stance legs
	if (guiIn.left_leg_hop) {
		isLeftStance = true;
		isRightStance = false;
	} else if (guiIn.right_leg_hop) {
		isLeftStance = false;
		isRightStance = true;
	} else if (guiIn.two_leg_hop) {
		isLeftStance = true;
		isRightStance = true;
	} else if (guiIn.alt_leg_hop) {
		// TODO - Add alternating leg capability
	}
	
	// If we are two leg hoppign the spring stiffness is twice as large.
	if (guiIn.two_leg_hop) {
		slipModel.ks = guiIn.slip_k*2.0;
	} else {
		slipModel.ks = guiIn.slip_k;
	}
	
    // Check and set current state
    if (rs.position.zPosition < guiIn.slip_l0) {
        isStance = true;
    } else {
        isStance = false;
    }

	// Leg controller ----------------------------------------------------------
	if (guiIn.leg_pos_contr) {
			// Set motor angles
			lMotorAngle = legToMotorPos(guiIn.left_leg_ang, guiIn.left_leg_len);
			rMotorAngle = legToMotorPos(guiIn.right_leg_ang, guiIn.right_leg_len);

			// Set motor currents
			co.lLeg.motorCurrentA = guiIn.leg_pos_kp*(lMotorAngle.A - rs.lLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfA.motorVelocity);
			co.lLeg.motorCurrentB = guiIn.leg_pos_kp*(lMotorAngle.B - rs.lLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfB.motorVelocity);
			co.rLeg.motorCurrentA = guiIn.leg_pos_kp*(rMotorAngle.A - rs.rLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfA.motorVelocity);
			co.rLeg.motorCurrentB = guiIn.leg_pos_kp*(rMotorAngle.B - rs.rLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfB.motorVelocity);

	} else if (guiIn.leg_slip_contr) {
	
		// Check if we are in stance phase.
		if (isStance) {
			
	       	// Compute SLIP force profile
        	slipConditions = slipAdvance0(slipModel, slipConditions);
			legForce = slipForce0(slipModel, slipConditions);
				
			// If we are two leg hopping each leg takes half the load.
			if (guiIn.two_leg_hop) {
				legForce.fx = legForce.fx/2.0;
				legForce.fz = legForce.fz/2.0;
				legForce.dfx = legForce.dfx/2.0;
				legForce.dfz = legForce.dfz/2.0;
			}	
			
			if (isLeftStance) {
				// If we think we should be in flight use position control.
				// If we think we are still in stance use force control.
				if (slipConditions.isFlight) {
					// Set motor angles
					lMotorAngle = legToMotorPos(leftLegAng, leftLegLen);

					// Set motor currents
					co.lLeg.motorCurrentA = guiIn.leg_pos_kp*(lMotorAngle.A - rs.lLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfA.motorVelocity);
					co.lLeg.motorCurrentB = guiIn.leg_pos_kp*(lMotorAngle.B - rs.lLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfB.motorVelocity);
					
				} else {
					// Compute current leg angle and length.		
    				leftLegLen = cos((rs.lLeg.halfB.legAngle - rs.lLeg.halfA.legAngle)/2.0);
    				leftLegAng = (rs.lLeg.halfA.legAngle + rs.lLeg.halfB.legAngle)/2.0;
					
      				// Compute and set motor current values
					motorCurrent = legForceToMotorCurrent0(legForce, gain, rs.lLeg, rs.position);
					co.lLeg.motorCurrentA = motorCurrent.A;
					co.lLeg.motorCurrentB = motorCurrent.B;
				}
				
			} else {
				// Set motor angles
				lMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_l0*0.75);

				// Set motor currents
				co.lLeg.motorCurrentA = guiIn.leg_pos_kp*(lMotorAngle.A - rs.lLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfA.motorVelocity);
				co.lLeg.motorCurrentB = guiIn.leg_pos_kp*(lMotorAngle.B - rs.lLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfB.motorVelocity);
			}
			
			if (isRightStance) {	        	
				// If we think we should be in flight use position control.
				// If we think we are still in stance use force control.
				if (slipConditions.isFlight) {
					// Set motor angles
					rMotorAngle = legToMotorPos(rightLegAng, rightLegLen);

					// Set motor currents
					co.rLeg.motorCurrentA = guiIn.leg_pos_kp*(rMotorAngle.A - rs.rLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfA.motorVelocity);
					co.rLeg.motorCurrentB = guiIn.leg_pos_kp*(rMotorAngle.B - rs.rLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfB.motorVelocity);
					
				} else {
					// Compute current leg angle and length.		
    				rightLegLen = cos((rs.rLeg.halfB.legAngle - rs.rLeg.halfA.legAngle)/2.0);
    				rightLegAng = (rs.rLeg.halfA.legAngle + rs.rLeg.halfB.legAngle)/2.0;
					
      				// Compute and set motor current values
					motorCurrent = legForceToMotorCurrent0(legForce, gain, rs.rLeg, rs.position);
					co.rLeg.motorCurrentA = motorCurrent.A;
					co.rLeg.motorCurrentB = motorCurrent.B;
				}
				
			} else {
				// Set motor angles
				rMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_l0*0.75);

				// Set motor currents
				co.rLeg.motorCurrentA = guiIn.leg_pos_kp*(rMotorAngle.A - rs.rLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfA.motorVelocity);
				co.rLeg.motorCurrentB = guiIn.leg_pos_kp*(rMotorAngle.B - rs.rLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfB.motorVelocity);
			}
			
	
		// Otherwise we must be in flight.
		} else {

		  	// Redefine slip initial conditions incase we go into stance next.
        	slipConditions.r = guiIn.slip_l0;
        	slipConditions.dr = -sqrt(2.0*9.81*guiIn.slip_h);
        	slipConditions.q = M_PI/2.0;
        	slipConditions.dq = 0.0;
			
			if (isLeftStance) {
				lMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_l0);
			} else {
				lMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_l0*0.75);
			}
			
			if (isRightStance) {
				rMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_l0);
			} else {
				rMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_l0*0.75);
			}

			// Set motor currents
			co.lLeg.motorCurrentA = guiIn.leg_pos_kp*(lMotorAngle.A - rs.lLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfA.motorVelocity);
			co.lLeg.motorCurrentB = guiIn.leg_pos_kp*(lMotorAngle.B - rs.lLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfB.motorVelocity);
			co.rLeg.motorCurrentA = guiIn.leg_pos_kp*(rMotorAngle.A - rs.rLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfA.motorVelocity);
			co.rLeg.motorCurrentB = guiIn.leg_pos_kp*(rMotorAngle.B - rs.rLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfB.motorVelocity);
		}
	
	} // leg controller
		
	
	// Hip controller ----------------------------------------------------------
	if (guiIn.hip_contr) {		

		// Get toe positions from GUI
		toePosition.left = guiIn.left_toe_pos;
		toePosition.right = guiIn.right_toe_pos;
		
		// Compute inverse kinematics
		hipAngle = inverseKinematics0(toePosition, rs.lLeg, rs.rLeg, rs.position);
		
        // Set motor currents
        co.lLeg.motorCurrentHip = guiIn.hip_pos_kp*(hipAngle.left - rs.lLeg.hip.legBodyAngle) + guiIn.hip_pos_kd*(0.0 - rs.lLeg.hip.legBodyVelocity);
        co.rLeg.motorCurrentHip = guiIn.hip_pos_kp*(hipAngle.right - rs.rLeg.hip.legBodyAngle) + guiIn.hip_pos_kd*(0.0 - rs.rLeg.hip.legBodyVelocity);
        
	} // hip controller
	
	

    // END CONTROL CODE ********************************************************

    // Command a run state
    co.command = medulla_state_run;

    // Let the GUI know the controller run state
    guiOut.isEnabled = ((rtOps::RtOpsState) rs.rtOpsState == rtOps::RtOpsState::ENABLED);

    // Send data to the GUI
    if (pubTimer->readyToSend()) guiDataOut.write(guiOut);

    // Output for RTOps
    return co;

} // ATCVerticalForceControlHopping::runController

// DON'T PUT CONTROL CODE BELOW HERE! ******************************************

// configureHook ===============================================================
bool ATCVerticalForceControlHopping::configureHook() {

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
	
	// ASCSlipModel Service
	ascSlipModel0 = this->getPeer(ascSlipModel0Name);
	if (ascSlipModel0) {
		slipAdvance0 = ascSlipModel0->provides("ascSlipModel")->getOperation("slipAdvance");
		slipForce0 = ascSlipModel0->provides("ascSlipModel")->getOperation("slipForce");
	}
	
	// Log controller data  
    log(Info) << "[ATCMT] configured!" << endlog();
    
    return true;

} // configureHook

// startHook ===================================================================
bool ATCVerticalForceControlHopping::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;

} // startHook

// updateHook ==================================================================
void ATCVerticalForceControlHopping::updateHook() {
    guiDataIn.read(guiIn);

} // updateHook

// stopHook ====================================================================
void ATCVerticalForceControlHopping::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();

} // stopHook

// cleanupHook =================================================================
void ATCVerticalForceControlHopping::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();

} // cleanupHook

ORO_CREATE_COMPONENT(ATCVerticalForceControlHopping)

} // namespace controller
} // namespace atrias
