/**
  * @file ATC_SLIP_HOPPING.cpp
  * @author Mikhail Jones
  * @brief This implements a SLIP template based vertical hopping controller.
  */

// TODO - Compute leg forces from spring deflection
// TODO - Use seperate ascLegForce controllers for each leg

#include "atc_slip_hopping/ATCSlipHopping.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// This constructor call is much simpler
ATCSlipHopping::ATCSlipHopping(string name) :
	ATC(name),
	ascCommonToolkit(this, "ascCommonToolkit"),
	ascSlipModel(this, "ascSlipModel"),
	ascLegForce(this, "ascLegForce"),
	ascHipBoomKinematics(this, "ascHipBoomKinematics"),
	ascPDlA(this, "ascPDlA"),
	ascPDlB(this, "ascPDlB"),
	ascPDrA(this, "ascPDrA"),
	ascPDrB(this, "ascPDrB"),
	ascPDlh(this, "ascPDlh"),
	ascPDrh(this, "ascPDrh")
{
	// Nothing to see here
}

/* @brief This is the main function for the top-level controller.
 * @param rs The robot state is an inherited member.
 * @param co The controller output is an inhereted member.
 */
void ATCSlipHopping::controller() {

	/* Additionally, the following functions are available to command the robot state:
	 * commandHalt();    // Trigger a Halt
	 * commandEStop();   // Trigger an EStop
	 * commandDisable(); // Disable the robot
	 * setStartupEnabled(true/false) // Enable or disable the default startup controller
	 * setShutdownEnabled(true/false) // Enable or disable the shutdown controller
	 */

    // Startup is handled by the ATC class
    setStartupEnabled(true);

    // Update current robot state
    updateState();

	// Run hip controller
	hipControl();

	// Main controller state machine
	switch (controllerState) {
		// Standing
		case 0:
			// Standing in place
			standingControl();
			break;

		// Vertical hopping
		case 1:
			// SLIP hopping controller state machine
			switch (hoppingState) {
				// Stance phase
				case 0:
					// Stance control type
					if (stanceControlType == 0) {
						// Run passive stance phase - zero torque at the hip
						passiveStancePhaseControl();
					} else if (stanceControlType == 1) {
						// Run force control stance phase - track SLIP forces
						forceStancePhaseControl();
					}
					break;

				// Flight phase
				case 1:
					// Run flight phase controller
					flightPhaseControl();
					break;
			}
			break;
	}

	// Copy over positions to the GUI output data
	guiOut.isEnabled = isEnabled();

}


// updateState
void ATCSlipHopping::updateState() {

	// Get GUI values
	controllerState = guiIn.main_controller;
	hoppingType = guiIn.hop_type;
	stanceControlType = guiIn.stance_controller;
	forceControlType = guiIn.force_type;
	springType = guiIn.spring_type;
	ascSlipModel.r0 = guiIn.slip_leg;
	h = guiIn.hop_height;

	// Set leg motor position control PD gains
	ascPDlA.P = ascPDlB.P = ascPDrA.P = ascPDrB.P = guiIn.leg_pos_kp;
	ascPDlA.D = ascPDlB.D = ascPDrA.D = ascPDrB.D = guiIn.leg_pos_kd;

	// Set hip motors position control PD gains
	ascPDlh.P = ascPDrh.P = guiIn.hip_pos_kp;
	ascPDlh.D = ascPDrh.D = guiIn.hip_pos_kd;

	// Set leg motor force control PID gains
	ascLegForce.kp = guiIn.leg_for_kp;
	ascLegForce.ki = 0.0;
	ascLegForce.kd = guiIn.leg_for_kd;

    // Check for stance phase and set hopping state
    if (rs.position.zPosition < ascSlipModel.r0) {
    	// Stance phase
        hoppingState = 0;
    } else {
        // Flight phase
        hoppingState = 1;
    }

	// Check hopping type and set stance leg(s)
	switch (hoppingType) {
		// Left leg hopping
		case 0:
			isLeftStance = true;
			isRightStance = false;
			break;
		// Right leg hopping
		case 1:
			isLeftStance = false;
			isRightStance = true;
			break;
		// Two leg hopping
		case 2:
			isLeftStance = true;
			isRightStance = true;
			break;
	}

}


// hipControl
void ATCSlipHopping::hipControl() {

	// Set toe positions
	toePosition.left = 2.15;
	toePosition.right = 2.45;

	// Compute inverse kinematics
	std::tie(qlh, qrh) = ascHipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);

	// Compute and set motor currents
	co.lLeg.motorCurrentHip = ascPDlh(qlh, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentHip = ascPDrh(qrh, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);

}


// standingControl
void ATCSlipHopping::standingControl() {

	// Set leg angles
	qll = qrl = PI/2.0;
	rll = rrl = guiIn.standing_leg;

	// Compute motor angles
	std::tie(qlmA, qlmB) = ascCommonToolkit.legPos2MotorPos(qll, rrl);
	std::tie(qrmA, qrmB) = ascCommonToolkit.legPos2MotorPos(qll, rrl);

	// Compute and set motor currents
	co.lLeg.motorCurrentA = ascPDlA(qlmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDlB(qlmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentA = ascPDrA(qrmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDrB(qrmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);

}


// forceStancePhaseControl
void ATCSlipHopping::forceStancePhaseControl() {

	// Spring type
	if (springType == 0) {
		ascSlipModel.k = ascCommonToolkit.legStiffness(slipState.r, ascSlipModel.r0);
	} else if (springType == 1) {
		ascSlipModel.k = guiIn.slip_spring;
	}

	// Set SLIP model parameters, double stiffness if two leg hopping
	if (hoppingType == 2) {
		ascSlipModel.k = 2.0*ascSlipModel.k;
	}

	// Compute SLIP force profile
	slipState = ascSlipModel.advanceRK5(slipState);
	legForce = ascSlipModel.force(slipState);

	// Halve the force if two leg hopping
	if (hoppingType == 2) {
		legForce.fx = legForce.fx/2.0;
		legForce.fz = legForce.fz/2.0;
		legForce.dfx = legForce.dfx/2.0;
		legForce.dfz = legForce.dfz/2.0;
	}

	// Left leg controller
	if (isLeftStance) {
		// If SLIP model says we should be in flight...
		if (slipState.isFlight) {
			// Use last know leg position from stance
			co.lLeg.motorCurrentA = ascPDlA(qlmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
			co.lLeg.motorCurrentB = ascPDlB(qlmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);

		// If SLIP model says we should be in stance...
		} else {
			// Store last known leg position
			qlmA = rs.lLeg.halfA.legAngle;
			qlmB = rs.lLeg.halfB.legAngle;

			// Compute and set motor currents
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForce.control(legForce, rs.lLeg, rs.position);
		}

	} else {
		// Set motor angles
		std::tie(qlmA, qlmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);

		// Compute and set motor currents
		co.lLeg.motorCurrentA = ascPDlA(qlmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
			co.lLeg.motorCurrentB = ascPDlB(qlmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	}

	// Right leg controller
	if (isRightStance) {

		// If SLIP model says we should be in flight...
		if (slipState.isFlight) {
			// Use last know leg position from stance
			co.rLeg.motorCurrentA = ascPDrA(qrmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
			co.rLeg.motorCurrentB = ascPDrB(qrmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);

		// If SLIP model says we should be in stance...
		} else {
			// Store last known leg position
			qrmA = rs.rLeg.halfA.legAngle;
			qrmB = rs.rLeg.halfB.legAngle;

			// Compute and set motor currents
			std::tie(co.rLeg.motorCurrentA, co.rLeg.motorCurrentB) = ascLegForce.control(legForce, rs.rLeg, rs.position);
		}

	} else {
		// Set motor angles
		std::tie(qrmA, qrmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);

		// Compute and set motor currents
		co.rLeg.motorCurrentA = ascPDrA(qrmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
		co.rLeg.motorCurrentB = ascPDrB(qrmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
	}
}


// passiveStancePhaseControl
void ATCSlipHopping::passiveStancePhaseControl() {

	// Left leg controller
	if (isLeftStance) {
		// Set motor angles
		std::tie(qlmA, qlmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0);
	} else {
		// Set motor angles
		std::tie(qlmA, qlmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);
	}

	// Right leg controller
	if (isRightStance) {
		// Set motor angles
		std::tie(qrmA, qrmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0);
	} else {
		// Set motor angles
		std::tie(qrmA, qrmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);
	}

	// Compute and set motor currents
	co.lLeg.motorCurrentA = ascPDlA(qlmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDlB(qlmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentA = ascPDrA(qrmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDrB(qrmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
}


// flightPhaseControl
void ATCSlipHopping::flightPhaseControl() {

	// Redefine slip initial conditions incase we go into stance next time step
	slipState.r = ascSlipModel.r0;
	slipState.q = PI/2.0;
	slipState.dq = 0.0;

	// Appex or terrain following force method
	switch (forceControlType) {
		// Appex tracking
		case 0:
			slipState.dr = rs.position.zVelocity;
			break;
		// Terrain following
		case 1:
			slipState.dr = -sqrt(2.0*9.81*h);
			break;
	}

	// Flight phase control
	if (isLeftStance) {
		// Set upcoming stance leg motor angles
		std::tie(qlmA, qlmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0);
	} else {
		// Set upcoming flight leg motor angles
		std::tie(qlmA, qlmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);
	}
	if (isRightStance) {
		// Set upcoming stance leg motor angles
		std::tie(qrmA, qrmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0);
	} else {
		// Set upcoming flight leg motor angles
		std::tie(qrmA, qrmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);
	}

	// Compute and set motor currents
	co.lLeg.motorCurrentA = ascPDlA(qlmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDlB(qlmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentA = ascPDrA(qrmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDrB(qrmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
}


// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCSlipHopping)

}
}
