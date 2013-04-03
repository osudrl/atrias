/**
  * @file ASC_LEG_FORCE_CONTROL.cpp
  * @author Mikhail Jones
  * @brief This implements a leg force controller (PD).
  */

// To use do something like this.
//
// // Define gain struct
// gain.kp = 1000.0;
// gain.kd = 8.0;
//
// // Define legForce struct
// legForce.fx = 0.0;
// legForce.fz = 0.0;
// legForce.dfx = 0.0;
// legForce.dfz = 0.0;
//
// // Compute and set required motorCurrent
// std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = legForceToMotorCurrent(legForce, gain, rs.lLeg, rs.position);

#include "asc_leg_force_control/ASCLegForceControl.hpp"

// Again, we need to put our code inside the appropriate namespaces.
namespace atrias {
namespace controller {

/* Breakdown of the next few lines:
 * We need to call our parent class's constructor,
 * then we can call the LogPort's constructor. The second parameter
 * is the name for this log port, which controls how it appears in the
 * bagfiles.
 */
ASCLegForceControl::ASCLegForceControl(AtriasController *parent, string name) :
        AtriasController(parent, name),
        log_out(this, "log")
{
	// No init required for this controller
}

// legForceToMotorCurrent
std::tuple<double, double> ASCLegForceControl::legForceToMotorCurrent(LegForce legForce, Gain gain, atrias_msgs::robot_state_leg leg, atrias_msgs::robot_state_location position) {

	// Unpack parameters
	fx = legForce.fx;
	fz = legForce.fz;
	dfx = legForce.dfx;
	dfz = legForce.dfz;
	qlA = leg.halfA.legAngle;
	qlB = leg.halfB.legAngle;
	qmA = leg.halfA.motorAngle;
	qmB = leg.halfB.motorAngle;
	qb = position.bodyPitch;
	dqlA = leg.halfA.legVelocity;
	dqlB = leg.halfB.legVelocity;
	dqmA = leg.halfA.motorVelocity;
	dqmB = leg.halfB.motorVelocity;
	dqb = position.bodyPitchVelocity;
	kp = gain.kp;
	kd = gain.kd;

	// Compute required joint torque using Jacobian
	tausA = -fx*l2*cos(qlA + qb) + fz*l2*sin(qlA + qb);
	tausB = -fx*l1*cos(qlB + qb) + fz*l1*sin(qlB + qb);

	// Compute required differential joint torque
	dtausA = fx*l2*sin(qlA + qb)*(dqlA + dqb) + dfz*l2*sin(qlA + qb) - dfx*l2*cos(qlA + qb) + fz*l2*cos(qlA + qb)*(dqlA + dqb);
	dtausB = fx*l1*sin(qlB + qb)*(dqlB + dqb) + dfz*l1*sin(qlB + qb) - dfx*l1*cos(qlB + qb) + fz*l1*cos(qlB + qb)*(dqlB + dqb);

	// Compute required motor current using PD controller with feed forward term
	motorCurrent.A = (ks*(qmA - qlA)/kg + kp*(tausA/ks - (qmA - qlA)) + kd*(dtausA/ks - (dqmA - dqlA)))/kt;
	motorCurrent.B = (ks*(qmB - qlB)/kg + kp*(tausB/ks - (qmB - qlB)) + kd*(dtausB/ks - (dqmB - dqlB)))/kt;
  
    // Set the log data
    log_out.data.fx = legForce.fx;
    log_out.data.fz = legForce.fz;
	log_out.data.tausA = tausA;
	log_out.data.tausB = tausB;
	log_out.data.dtausA = dtausA;
	log_out.data.dtausB = dtausB;

    // Transmit the log data
    log_out.send();

	// Return the motor current
	return std::make_tuple(motorCurrent.A, motorCurrent.B);

} // legForceToMotorCurrent

}
}
