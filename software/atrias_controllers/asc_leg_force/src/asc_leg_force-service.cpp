// Title: ASCLegForce
// Description: Defines leg force to motor torque relationships.
// Author: Mikhail Jones

// Usage: To use this subcontroller, add the following pieces of code to the the designated files in your controller folder.

// Manifest.xml
// <depend package="asc_leg_force"/>

// Start.ops
// # ASCLegForce
// require("ascLegForce")
// loadService("controller", "ascLegForce")

// component_controller.cpp
// // ASCLegForce Service
// legForceToMotorCurrent = this->provides("ascLegForce")->getOperation("legForceToMotorCurrent");

// component_controller.h
// // ASCLegForce
// OperationCaller<MotorCurrent(LegForce, Gain, atrias_msgs::robot_state_leg, atrias_msgs::robot_state_location)> legForceToMotorCurrent;

// To use do something like this.
//
// // Define gain struct
// gain.kp = 1000.0;
// gain.kd = 8.0;
// gain.ks = 4118.0;
// gain.kg = 50.0;    
// gain.kt = 0.0987;
//
// // Define legForce struct.
// legForce.fx = 0.0;
// legForce.fz = 0.0;
// legForce.dfx = 0.0;
// legForce.dfz = 0.0;
//
// // Compute and set required motorCurrent
// motorCurrent = legForceToMotorCurrent(legForce, gain, rs.lLeg, rs.position);
// co.rLeg.motorCurrentA = motorCurrent.A;
// co.rLeg.motorCurrentB = motorCurrent.B;

#include <asc_leg_force/asc_leg_force-service.h>

namespace atrias {
namespace controller {

// ASCLegForce Constructor =====================================================
ASCLegForce::ASCLegForce(TaskContext* owner):Service("ascLegForce", owner) {
    this->addOperation("legForceToMotorCurrent", &ASCLegForce::legForceToMotorCurrent, this).doc("Given a desired X-Z component force, returns the required motor current.");
}


// ASCLegForce legForce ========================================================
AB ASCLegForce::legForceToMotorCurrent(LegForce legForce, Gain gain, atrias_msgs::robot_state_leg leg, atrias_msgs::robot_state_location position) {

	// Declare robot parameters.
	l1 = 0.50;
	l2 = 0.50;

	// Compute required joint torque using Jacobian.
	tauSpringA = -legForce.fx*l2*cos(leg.halfA.legAngle + position.bodyPitch) + legForce.fz*l2*sin(leg.halfA.legAngle + position.bodyPitch);
	tauSpringB = -legForce.fx*l1*cos(leg.halfB.legAngle + position.bodyPitch) + legForce.fz*l1*sin(leg.halfB.legAngle + position.bodyPitch);

	// Compute required differential joint torque.
	dtauSpringA = legForce.fx*l2*sin(leg.halfA.legAngle + position.bodyPitch)*(leg.halfA.legVelocity + position.bodyPitchVelocity) + legForce.dfz*l2*sin(leg.halfA.legAngle + position.bodyPitch) - legForce.dfx*l2*cos(leg.halfA.legAngle + position.bodyPitch) + legForce.fz*l2*cos(leg.halfA.legAngle + position.bodyPitch)*(leg.halfA.legVelocity + position.bodyPitchVelocity);
	dtauSpringB = legForce.fx*l1*sin(leg.halfB.legAngle + position.bodyPitch)*(leg.halfB.legVelocity + position.bodyPitchVelocity) + legForce.dfz*l1*sin(leg.halfB.legAngle + position.bodyPitch) - legForce.dfx*l1*cos(leg.halfB.legAngle + position.bodyPitch) + legForce.fz*l1*cos(leg.halfB.legAngle + position.bodyPitch)*(leg.halfB.legVelocity + position.bodyPitchVelocity);

	// Compute required motor current using PD controller with feed forward term.
	motorCurrent.A = (gain.ks*(leg.halfA.motorAngle - leg.halfA.legAngle)/gain.kg + gain.kp*(tauSpringA/gain.ks - (leg.halfA.motorAngle - leg.halfA.legAngle)) + gain.kd*(dtauSpringA/gain.ks - (leg.halfA.motorVelocity - leg.halfA.legVelocity)))/gain.kt;
	motorCurrent.B = (gain.ks*(leg.halfB.motorAngle - leg.halfB.legAngle)/gain.kg + gain.kp*(tauSpringB/gain.ks - (leg.halfB.motorAngle - leg.halfB.legAngle)) + gain.kd*(dtauSpringB/gain.ks - (leg.halfB.motorVelocity - leg.halfB.legVelocity)))/gain.kt;

	// DEBUG STATEMENTS (Not real-time safe)
	//printf("tauA: %f\n", tauSpringA);
	//printf("taub: %f\n", tauSpringB);
	//printf("dtauA: %f\n", dtauSpringA);
	//printf("dtaub: %f\n", dtauSpringB);

	return motorCurrent;

} // legForceToMotorCurrent

ORO_SERVICE_NAMED_PLUGIN(ASCLegForce, "ascLegForce")

} // namespace controller
} // namespace atrias
