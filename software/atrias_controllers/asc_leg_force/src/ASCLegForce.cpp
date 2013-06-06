#include "asc_leg_force/ASCLegForce.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

ASCLegForce::ASCLegForce(AtriasController *parent, string name) :
        AtriasController(parent, name),
        log_out(this, "log")
{
	// PID controller gains
	kp = 250.0;
	ki = 0.0;
	kd = 5.0;
	
	// Anti wind-up limit for integral error term
	antiWindup = 5.0;
}


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

	// Compute required joint torque from desired end effector forces using Jacobian
	tausA = -fx*L2*cos(qlA + qb) + fz*L2*sin(qlA + qb);
	tausB = -fx*L1*cos(qlB + qb) + fz*L1*sin(qlB + qb);

	// Compute required differential joint torque
	dtausA = fx*L2*sin(qlA + qb)*(dqlA + dqb) + dfz*L2*sin(qlA + qb) - dfx*L2*cos(qlA + qb) + fz*L2*cos(qlA + qb)*(dqlA + dqb);
	dtausB = fx*L1*sin(qlB + qb)*(dqlB + dqb) + dfz*L1*sin(qlB + qb) - dfx*L1*cos(qlB + qb) + fz*L1*cos(qlB + qb)*(dqlB + dqb);

	// Compute proportional error terms
	epA = tausA/KS - (qmA - qlA);
	epB = tausB/KS - (qmB - qlB);
	
	// Compute integral error terms using clamping anti-windup method
	eiA = clamp(eiA + (epA * 0.001), -antiWindup, antiWindup);
	eiB = clamp(eiB + (epB * 0.001), -antiWindup, antiWindup);
	
	// Compute derivative error terms
	edA = dtausA/KS - (dqmA - dqlA);
	edB = dtausB/KS - (dqmB - dqlB);
	
	// Compute required motor current using PD terms on spring deflection with feed forward term
	curA = (tausA/KG + kp*epA + ki*eiA + kd*edA)/KT;
	curB = (tausB/KG + kp*epB + ki*eiB + kd*edB)/KT;
    	
	// Set the log data
	log_out.data.control_fx = legForce.fx;
	log_out.data.control_fz = legForce.fz;
	log_out.data.control_dfx = legForce.dfx;
	log_out.data.control_dfz = legForce.dfz;
	log_out.data.control_tausA = tausA;
	log_out.data.control_tausB = tausB;
	log_out.data.control_dtausA = dtausA;
	log_out.data.control_dtausB = dtausB;
	log_out.data.control_curA = curA;
	log_out.data.control_curB = curB;

	// Transmit the log data
	log_out.send();

	// Return the motor current
	return std::make_tuple(curA, curB);
}


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

	// Compute the spring torques derivative
	dtausA = KS*(dqmA - dqlA);
	dtausB = KS*(dqmB - dqlB);

	// Compute the leg forces
	legForce.fx = -(L2*tausB*sin(qb + qlA) - L1*tausA*sin(qb + qlB))/(L1*L2*sin(qlA - qlB));
	legForce.fz = -(L2*tausB*cos(qb + qlA) - L1*tausA*cos(qb + qlB))/(L1*L2*sin(qlA - qlB));

	// Compute the leg forces derivative
	legForce.dfx = -(L2*dtausB*sin(qb + qlA) - L1*dtausA*sin(qb + qlB) + L2*tausB*cos(qb + qlA)*(dqb + dqlA) - L1*tausA*cos(qb + qlB)*(dqb + dqlB))/(L1*L2*sin(qlA - qlB)) + (cos(qlA - qlB)/pow(sin(qlA - qlB), 2)*(L2*tausB*sin(qb + qlA) - L1*tausA*sin(qb + qlB))*(dqlA - dqlB))/(L1*L2);
	legForce.dfz = -(L2*dtausB*cos(qb + qlA) - L1*dtausA*cos(qb + qlB) - L2*tausB*sin(qb + qlA)*(dqb + dqlA) + L1*tausA*sin(qb + qlB)*(dqb + dqlB))/(L1*L2*sin(qlA - qlB)) + (cos(qlA - qlB)/pow(sin(qlA - qlB), 2)*(L2*tausB*cos(qb + qlA) - L1*tausA*cos(qb + qlB))*(dqlA - dqlB))/(L1*L2);

	// Set the log data
	log_out.data.compute_tausA = tausA;
	log_out.data.compute_tausB = tausB;
	log_out.data.compute_dtausA = dtausA;
	log_out.data.compute_dtausB = dtausB;
	log_out.data.compute_fx = legForce.fx;
	log_out.data.compute_fz = legForce.fz;
	log_out.data.compute_dfx = legForce.dfx;
	log_out.data.compute_dfz = legForce.dfz;

	// Transmit the log data
	log_out.send();

	// Return the leg forces
	return legForce;
}

}
}
