/*! \file controller_component.cpp
 *  \author Mikhail Jones
 *  \brief Orocos Component code for the atc_force_control_hopping controller.
 */

// Initialize ==================================================================
#include <atc_force_control_hopping/controller_component.h>

namespace atrias {
namespace controller {

// ATCForceControlHopping ======================================================
ATCForceControlHopping::ATCForceControlHopping(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log"),
    guiDataOut("gui_data_out"),
    guiDataIn("gui_data_in")
{
	// Operations --------------------------------------------------------------
    this->provides("atc")->addOperation("runController", &ATCForceControlHopping::runController, this, ClientThread).doc("Get robot_state from RTOps and return controller output.");

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

	// Variables ---------------------------------------------------------------
	isStance = false;
    isLeftStance = false;
    isRightStance = false;
    
    isInitialize = true;
    isDeinitialize = true;
    isFinished = false;
    duration = 5.0;
    t = 0.0;

    // Logging -----------------------------------------------------------------
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


// ATCForceControlHopping ======================================================
atrias_msgs::controller_output ATCForceControlHopping::runController(atrias_msgs::robot_state rs) {

    // Do nothing unless told otherwise
    co.lLeg.motorCurrentA = 0.0;
    co.lLeg.motorCurrentB = 0.0;
    co.lLeg.motorCurrentHip = 0.0;
    co.rLeg.motorCurrentA = 0.0;
    co.rLeg.motorCurrentB = 0.0;
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
	
	// Set SLIP parameters
	slipModel.g = -9.81;
	slipModel.m = guiIn.slip_m;
	slipModel.r0 = guiIn.slip_r0;
	
	// If we are two leg hopping the spring stiffness is twice as large
	if (guiIn.two_leg) {
		slipModel.k = 2.0*guiIn.robot_ks;
	} else {
		slipModel.k = guiIn.robot_ks;
	}
	
	// Decide which are stance legs
	if (guiIn.left_leg) {
		isLeftStance = true;
		isRightStance = false;
	} else if (guiIn.right_leg) {
		isLeftStance = false;
		isRightStance = true;
	} else if (guiIn.two_leg) {
		isLeftStance = true;
		isRightStance = true;
	} else if (guiIn.alt_leg) {
		// TODO - Add alternating leg capability
	}
	
    // Check and set current state
    // TODO - Base on toe switch
    if (rs.position.zPosition < guiIn.slip_r0) {
        isStance = true;
    } else {
        isStance = false;
    }

	// Leg controller ----------------------------------------------------------
	if (guiIn.stand) {
			// Set motor angles
			lMotorAngle = legToMotorPos(M_PI/2.0, guiIn.stand_r0);
			rMotorAngle = legToMotorPos(M_PI/2.0, guiIn.stand_r0);

			// Set motor currents
			co.lLeg.motorCurrentA = guiIn.leg_pos_kp*(lMotorAngle.A - rs.lLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfA.motorVelocity);
			co.lLeg.motorCurrentB = guiIn.leg_pos_kp*(lMotorAngle.B - rs.lLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfB.motorVelocity);
			co.rLeg.motorCurrentA = guiIn.leg_pos_kp*(rMotorAngle.A - rs.rLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfA.motorVelocity);
			co.rLeg.motorCurrentB = guiIn.leg_pos_kp*(rMotorAngle.B - rs.rLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfB.motorVelocity);

	} else if (guiIn.hop) {
	
		// Check if we are in stance phase
		if (isStance) {
			
	       	// Compute SLIP force profile
        	slipState = slipAdvance0(slipModel, slipState);
			legForce = slipForce0(slipModel, slipState);
				
			// If we are two leg hopping each leg takes half the load
			if (guiIn.two_leg) {
				legForce.fx = legForce.fx/2.0;
				legForce.fz = legForce.fz/2.0;
				legForce.dfx = legForce.dfx/2.0;
				legForce.dfz = legForce.dfz/2.0;
			}	
			
			if (isLeftStance) {
				// If we think we should be in flight use position control
				// If we think we are still in stance use force control
				if (slipState.isFlight) {
					// Set motor angles
					lMotorAngle = legToMotorPos(leftLegAng, leftLegLen);

					// Set motor currents
					co.lLeg.motorCurrentA = guiIn.leg_pos_kp*(lMotorAngle.A - rs.lLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfA.motorVelocity);
					co.lLeg.motorCurrentB = guiIn.leg_pos_kp*(lMotorAngle.B - rs.lLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfB.motorVelocity);
					
				} else {
					// Compute current leg angle and length	
    				leftLegLen = cos((rs.lLeg.halfB.legAngle - rs.lLeg.halfA.legAngle)/2.0);
    				leftLegAng = (rs.lLeg.halfA.legAngle + rs.lLeg.halfB.legAngle)/2.0;
					
      				// Compute and set motor current values
					motorCurrent = legForceToMotorCurrent0(legForce, gain, rs.lLeg, rs.position);
					co.lLeg.motorCurrentA = motorCurrent.A;
					co.lLeg.motorCurrentB = motorCurrent.B;
				}
				
			} else {
				// Set motor angles
				lMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_r0*0.75);

				// Set motor currents
				co.lLeg.motorCurrentA = guiIn.leg_pos_kp*(lMotorAngle.A - rs.lLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfA.motorVelocity);
				co.lLeg.motorCurrentB = guiIn.leg_pos_kp*(lMotorAngle.B - rs.lLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfB.motorVelocity);
			}
			
			if (isRightStance) {	        	
				// If we think we should be in flight use position control
				// If we think we are still in stance use force control
				if (slipState.isFlight) {
					// Set motor angles
					rMotorAngle = legToMotorPos(rightLegAng, rightLegLen);

					// Set motor currents
					co.rLeg.motorCurrentA = guiIn.leg_pos_kp*(rMotorAngle.A - rs.rLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfA.motorVelocity);
					co.rLeg.motorCurrentB = guiIn.leg_pos_kp*(rMotorAngle.B - rs.rLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfB.motorVelocity);
					
				} else {
					// Compute current leg angle and length		
    				rightLegLen = cos((rs.rLeg.halfB.legAngle - rs.rLeg.halfA.legAngle)/2.0);
    				rightLegAng = (rs.rLeg.halfA.legAngle + rs.rLeg.halfB.legAngle)/2.0;
					
      				// Compute and set motor current values
					motorCurrent = legForceToMotorCurrent0(legForce, gain, rs.rLeg, rs.position);
					co.rLeg.motorCurrentA = motorCurrent.A;
					co.rLeg.motorCurrentB = motorCurrent.B;
				}
				
			} else {
				// Set motor angles
				rMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_r0*0.75);

				// Set motor currents
				co.rLeg.motorCurrentA = guiIn.leg_pos_kp*(rMotorAngle.A - rs.rLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfA.motorVelocity);
				co.rLeg.motorCurrentB = guiIn.leg_pos_kp*(rMotorAngle.B - rs.rLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfB.motorVelocity);
			}
			
	
		// Otherwise we must be in flight
		} else {

		  	// Redefine slip initial conditions incase we go into stance next
        	slipState.r = guiIn.slip_r0;
        	slipState.q = M_PI/2.0;
        	slipState.dq = 0.0;
        	
        	// Appex or terrain following force method
        	// TODO - Only goo for vertical hopping, add velocity components
        	if (guiIn.appex) {
        		slipState.dr = rs.position.zVelocity;
        	} else if (guiIn.terrain) {
        		slipState.dr = -sqrt(2.0*9.81*guiIn.slip_h);
        	}
			
			if (isLeftStance) {
				lMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_r0);
			} else {
				lMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_r0*0.75);
			}
			
			if (isRightStance) {
				rMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_r0);
			} else {
				rMotorAngle = legToMotorPos(M_PI/2.0, guiIn.slip_r0*0.75);
			}

			// Set motor currents
			co.lLeg.motorCurrentA = guiIn.leg_pos_kp*(lMotorAngle.A - rs.lLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfA.motorVelocity);
			co.lLeg.motorCurrentB = guiIn.leg_pos_kp*(lMotorAngle.B - rs.lLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfB.motorVelocity);
			co.rLeg.motorCurrentA = guiIn.leg_pos_kp*(rMotorAngle.A - rs.rLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfA.motorVelocity);
			co.rLeg.motorCurrentB = guiIn.leg_pos_kp*(rMotorAngle.B - rs.rLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.rLeg.halfB.motorVelocity);
		}
	
	} // leg controller
		
	
	// Hip controller ----------------------------------------------------------
	if (guiIn.hip) {		

		// Get toe positions from GUI
		toePosition.left = guiIn.left_toe;
		toePosition.right = guiIn.right_toe;
		
		// Compute inverse kinematics
		hipAngle = inverseKinematics0(toePosition, rs.lLeg, rs.rLeg, rs.position);
		
        // Set motor currents
        co.lLeg.motorCurrentHip = guiIn.hip_kp*(hipAngle.left - rs.lLeg.hip.legBodyAngle) + guiIn.hip_kd*(0.0 - rs.lLeg.hip.legBodyVelocity);
        co.rLeg.motorCurrentHip = guiIn.hip_kp*(hipAngle.right - rs.rLeg.hip.legBodyAngle) + guiIn.hip_kd*(0.0 - rs.rLeg.hip.legBodyVelocity);
        
	} // hip controller



	// TODO - Turn into state machine and break out sections of code into functions
	// When first starting, slowly ramp up commanded motor currents
	if (isInitialize) {
		t = t + 0.001;
    	co.lLeg.motorCurrentA = co.lLeg.motorCurrentA*t/duration;
    	co.lLeg.motorCurrentB = co.lLeg.motorCurrentB*t/duration;
    	co.lLeg.motorCurrentHip = co.lLeg.motorCurrentHip*t/duration;
    	co.rLeg.motorCurrentA = co.rLeg.motorCurrentA*t/duration;
    	co.rLeg.motorCurrentB = co.rLeg.motorCurrentB*t/duration;
    	co.rLeg.motorCurrentHip = co.rLeg.motorCurrentHip*t/duration;	
		if (t > duration) {
			isInitialize = false;
			t = 0.0;
		} 	
	}
	
	if (isDeinitialize && guiIn.deinit) {
		t = t + 0.001;
    	co.lLeg.motorCurrentA = co.lLeg.motorCurrentA*(1.0 - t/duration);
    	co.lLeg.motorCurrentB = co.lLeg.motorCurrentB*(1.0 - t/duration);
    	co.lLeg.motorCurrentHip = co.lLeg.motorCurrentHip*(1.0 - t/duration);
    	co.rLeg.motorCurrentA = co.rLeg.motorCurrentA*(1.0 - t/duration);
    	co.rLeg.motorCurrentB = co.rLeg.motorCurrentB*(1.0 - t/duration);
    	co.rLeg.motorCurrentHip = co.rLeg.motorCurrentHip*(1.0 - t/duration);	
		if (t > duration) {
			isDeinitialize = false;
			isFinished = true;
			t = 0.0;
		} 	
	}
	
	if (isFinished) {
	    co.lLeg.motorCurrentA = 0.0;
		co.lLeg.motorCurrentB = 0.0;
		co.lLeg.motorCurrentHip = 0.0;
		co.rLeg.motorCurrentA = 0.0;
		co.rLeg.motorCurrentB = 0.0;
		co.rLeg.motorCurrentHip = 0.0;
    }
	

    // END CONTROL CODE ********************************************************

    // Command a run state
    co.command = medulla_state_run;

    // Let the GUI know the controller run state
    guiOut.isEnabled = ((rtOps::RtOpsState) rs.rtOpsState == rtOps::RtOpsState::ENABLED);
    
    // Send data to the GUI
    if (pubTimer->readyToSend()) guiDataOut.write(guiOut);

    // Stuff the msg and push to ROS for logging
    //logPort.write(logData);

    // Output for RTOps
    return co;
    
} // runController

// DON'T PUT CONTROL CODE BELOW HERE! ******************************************

// configureHook ===============================================================
bool ATCForceControlHopping::configureHook() {

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
	
    log(Info) << "[ATCMT] configured!" << endlog();
    return true;
    
} // configureHook

// startHook ===================================================================
bool ATCForceControlHopping::startHook() {
    log(Info) << "[ATCMT] started!" << endlog();
    return true;
    
} // startHook

// updateHook ==================================================================
void ATCForceControlHopping::updateHook() {
    guiDataIn.read(guiIn);
    
} // updateHook

// stopHook ====================================================================
void ATCForceControlHopping::stopHook() {
    log(Info) << "[ATCMT] stopped!" << endlog();
    
} // stopHook

// cleanupHook =================================================================
void ATCForceControlHopping::cleanupHook() {
    log(Info) << "[ATCMT] cleaned up!" << endlog();
    
} // cleanupHook

ORO_CREATE_COMPONENT(ATCForceControlHopping)

} // namespace controller
} // namespace atrias
