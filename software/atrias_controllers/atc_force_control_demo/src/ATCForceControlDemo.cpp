/**
  * @file ATC_FORC_CONTROL_DEMO.hpp
  * @author Mikhail Jones
  * @brief This implements a demo force controller.
  */
  
#include "atc_force_control_demo/ATCForceControlDemo.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// This constructor call is much simpler.
ATCForceControlDemo::ATCForceControlDemo(string name) :
	ATC(name),
	ascCommonToolkit(this, "ascCommonToolkit"),
	ascSlipModel(this, "ascSlipModel"),
	ascLegForce(this, "ascLegForce"),
	ascHipBoomKinematics(this, "ascHipBoomKinematics")
{
	// Initialize
	t = 0.0;
}

/* @brief This is the main function for the top-level controller.
 * @param rs The robot state is an inherited member.
 * @param co The controller output is an inhereted member.
 */
void ATCForceControlDemo::controller() {
	
	/* Additionally, the following functions are available to command the robot state:
	 * commandHalt();    // Trigger a Halt
	 * commandEStop();   // Trigger an EStop
	 * commandDisable(); // Disable the robot
	 * setStartupEnabled(true/false) // Enable or disable the default startup controller
	 * setShutdownEnabled(true/false) // Enable or disable the shutdown controller
	 */
	
	// Startup is handled by the ATC class.
	// setStartupEnabled(true);

	// Initialize variables
	legForce.fx = 0.0;
	legForce.fz = 0.0;
	legForce.dfx = 0.0;
	legForce.dfz = 0.0;
	ascLegForce.kp = guiIn.leg_for_kp;
	ascLegForce.kd = guiIn.leg_for_kd;

	// Increment time step
	t = t + 0.001;


	// Left leg controller
	if (guiIn.left_leg_pos) {		
		// Set motor angles
		std::tie(lMotorAngle.A, lMotorAngle.B) = ascCommonToolkit.legPos2MotorPos(guiIn.left_leg_ang, guiIn.left_leg_len);

		// Set motor currents
		co.lLeg.motorCurrentA = guiIn.leg_pos_kp*(lMotorAngle.A - rs.lLeg.halfA.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfA.motorVelocity);
		co.lLeg.motorCurrentB = guiIn.leg_pos_kp*(lMotorAngle.B - rs.lLeg.halfB.motorAngle) + guiIn.leg_pos_kd*(0.0 - rs.lLeg.halfB.motorVelocity);

	} else if (guiIn.left_leg_for) {
		// Get component forces from GUI, velocities equal to zero.
		legForce.fx = guiIn.left_fx;
		legForce.fz = guiIn.left_fz;
		legForce.dfx = 0.0;
		legForce.dfz = 0.0;
		
		// Compute and set motor current values
		std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForce.control(legForce, rs.lLeg, rs.position);
		
	} else if (guiIn.left_leg_wave_for) {
		// Compute sinewave forces
		legForce.fx = guiIn.left_offx + guiIn.left_ampx*sin(t*2.0*PI*guiIn.left_freqx);
		legForce.fz = guiIn.left_offz + guiIn.left_ampz*sin(t*2.0*PI*guiIn.left_freqz);
		legForce.dfx = 2.0*PI*guiIn.left_ampx*guiIn.left_freqx*cos(t*2.0*PI*guiIn.left_freqx);
		legForce.dfz = 2.0*PI*guiIn.left_ampz*guiIn.left_freqz*cos(t*2.0*PI*guiIn.left_freqz);;
		
		// Comute and set motor current values
		std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForce.control(legForce, rs.lLeg, rs.position);
		
	}


	// Right leg controller
	if (guiIn.right_leg_pos) {		
		// Set motor angles
		std::tie(rMotorAngle.A, rMotorAngle.B) = ascCommonToolkit.legPos2MotorPos(guiIn.right_leg_ang, guiIn.right_leg_len);

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
		std::tie(co.rLeg.motorCurrentA, co.rLeg.motorCurrentB) = ascLegForce.control(legForce, rs.rLeg, rs.position);
		
	} else if (guiIn.right_leg_wave_for) {
		// Compute sinewave forces
		legForce.fx = guiIn.right_offx + guiIn.right_ampx*sin(t*2.0*PI*guiIn.right_freqx);
		legForce.fz = guiIn.right_offz + guiIn.right_ampz*sin(t*2.0*PI*guiIn.right_freqz);
		legForce.dfx = 2.0*PI*guiIn.right_ampx*guiIn.right_freqx*cos(t*2.0*PI*guiIn.right_freqx);
		legForce.dfz = 2.0*PI*guiIn.right_ampz*guiIn.right_freqz*cos(t*2.0*PI*guiIn.right_freqz);;
		
		// Comute and set motor current values
		std::tie(co.rLeg.motorCurrentA, co.rLeg.motorCurrentB) = ascLegForce.control(legForce, rs.rLeg, rs.position);
		
	}

	
	// Hip controller
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
		std::tie(hipAngle.left, hipAngle.right) = ascHipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);
		
		// Set motor currents
		co.lLeg.motorCurrentHip = guiIn.hip_pos_kp*(hipAngle.left - rs.lLeg.hip.legBodyAngle) + guiIn.hip_pos_kd*(0.0 - rs.lLeg.hip.legBodyVelocity);
		co.rLeg.motorCurrentHip = guiIn.hip_pos_kp*(hipAngle.right - rs.rLeg.hip.legBodyAngle) + guiIn.hip_pos_kd*(0.0 - rs.rLeg.hip.legBodyVelocity);

	}

	// Copy over positions to the GUI output data
	guiOut.isEnabled = isEnabled();
	
}


// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCForceControlDemo)

}
}
