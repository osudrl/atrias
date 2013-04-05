/**
  * @file ASC_LEG_FORCE.cpp
  * @author Mikhail Jones
  * @brief This implements leg force functions.
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
// std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForce.control(legForce, rs.lLeg, rs.position);

#include "asc_leg_force/ASCLegForce.hpp"

// Again, we need to put our code inside the appropriate namespaces.
namespace atrias {
namespace controller {

/* Breakdown of the next few lines:
 * We need to call our parent class's constructor,
 * then we can call the LogPort's constructor. The second parameter
 * is the name for this log port, which controls how it appears in the
 * bagfiles.
 */
ASCLegForce::ASCLegForce(AtriasController *parent, string name) :
        AtriasController(parent, name),
        log_out(this, "log")
{
	// Initialize
	kp = 1000.0;
	ki = 0.0;
	kd = 8.0;
}

// control
std::tuple<double, double> ASCLegForce::control(LegForce legForce, atrias_msgs::robot_state_leg leg, atrias_msgs::robot_state_location position) {

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

	// Compute required joint torque using Jacobian
	tausA = -fx*L2*cos(qlA + qb) + fz*L2*sin(qlA + qb);
	tausB = -fx*L1*cos(qlB + qb) + fz*L1*sin(qlB + qb);

	// Compute required differential joint torque
	dtausA = fx*L2*sin(qlA + qb)*(dqlA + dqb) + dfz*L2*sin(qlA + qb) - dfx*L2*cos(qlA + qb) + fz*L2*cos(qlA + qb)*(dqlA + dqb);
	dtausB = fx*L1*sin(qlB + qb)*(dqlB + dqb) + dfz*L1*sin(qlB + qb) - dfx*L1*cos(qlB + qb) + fz*L1*cos(qlB + qb)*(dqlB + dqb);


	// Compute required motor current using PD controller with feed forward term
	curA = (KS*(qmA - qlA)/KG + kp*(tausA/KS - (qmA - qlA)) + kd*(dtausA/KS - (dqmA - dqlA)))/KT;
	curB = (KS*(qmB - qlB)/KG + kp*(tausB/KS - (qmB - qlB)) + kd*(dtausB/KS - (dqmB - dqlB)))/KT;
  
    // Set the log data
	log_out.data.tausA = tausA;
	log_out.data.tausB = tausB;
	log_out.data.dtausA = dtausA;
	log_out.data.dtausB = dtausB;

    // Transmit the log data
    log_out.send();

	// Return the motor current
	return std::make_tuple(curA, curB);

} // control

// compute
LegForce ASCLegForce::compute(atrias_msgs::robot_state_leg leg, atrias_msgs::robot_state_location position) {

	// Unpack the parameters
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
	
	// Compute the spring torques
	tausA = KS*(qmA - qlA);
	tausB = KS*(qmB - qlB);
	
	// Compute the derivative of the spring torques
	dtausA = KS*(dqmA - dqlA);
	dtausB = KS*(dqmB - dqlB);
	
	// Compute the leg forces
	legForce.fx = -(L2*tausB*sin(qb + qlA) - L1*tausA*sin(qb + qlB))/(L1*L2*sin(qlA - qlB));
	legForce.fz = -(L2*tausB*cos(qb + qlA) - L1*tausA*cos(qb + qlB))/(L1*L2*sin(qlA - qlB));
	
	// Compute the derivative of the leg forces
	legForce.dfx = -(L2*dtausB*sin(qb + qlA) - L1*dtausA*sin(qb + qlB) + L2*tausB*cos(qb + qlA)*(dqb + dqlA) - L1*tausA*cos(qb + qlB)*(dqb + dqlB))/(L1*L2*sin(qlA - qlB)) + (cos(qlA - qlB)*1.0/pow(sin(qlA - qlB), 2)*(L2*tausB*sin(qb + qlA) - L1*tausA*sin(qb + qlB))*(dqlA - dqlB))/(L1*L2);
	legForce.dfz = -(L2*dtausB*cos(qb + qlA) - L1*dtausA*cos(qb + qlB) - L2*tausB*sin(qb + qlA)*(dqb + dqlA) + L1*tausA*sin(qb + qlB)*(dqb + dqlB))/(L1*L2*sin(qlA - qlB)) + (cos(qlA - qlB)*1.0/pow(sin(qlA - qlB), 2)*(L2*tausB*cos(qb + qlA) - L1*tausA*cos(qb + qlB))*(dqlA - dqlB))/(L1*L2);

    // Set the log data
	log_out.data.fx = legForce.fx;
	log_out.data.fz = legForce.fz;
	log_out.data.dfx = legForce.dfx;
	log_out.data.dfz = legForce.dfz;

    // Transmit the log data
    log_out.send();
    
	// Return the leg forces
	return legForce;

} // compute

}
}
