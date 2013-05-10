#include "asc_common_toolkit/ASCCommonToolkit.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

ASCCommonToolkit::ASCCommonToolkit(AtriasController *parent, string name) :
        AtriasController(parent, name),
        log_out(this, "log")
{
	// Nothing to see here
}


std::tuple<double, double> ASCCommonToolkit::legStiffness(double r, double dr, double r0) {

	// Compute non-linear ATRIAS virtual leg length stiffness
	k = KS*(sin(acos(r)) - (acos(r) - acos(r0))*cos(acos(r)))/(2.0*L1*L2*pow(sin(acos(r)), 3));
	dk = -(KS*dr*(acos(r) - acos(r0)))/(2.0*L1*L2*pow((1.0 - r*r), 1.5)) - (3.0*KS*dr*r*(r*(acos(r) - acos(r0)) - pow((1.0 - r*r), 0.5)))/(2.0*L1*L2*pow((1.0 - r*r), 2.5));

	// Set the log data
	log_out.data.k = k;
	log_out.data.dk = dk;

	// Transmit the log data
	log_out.send();

	// Return virtual leg stiffness
	return std::make_tuple(k, dk);
	
}


std::tuple<double, double> ASCCommonToolkit::motorPos2LegPos(double qmA, double qmB) {

	// Compute leg positions
	ql = ((qmA + qmB)/2.0);
	rl = cos((qmA - qmB)/2.0);
	
	// Return leg position
	return std::make_tuple(ql, rl);
	
}


std::tuple<double, double> ASCCommonToolkit::legPos2MotorPos(double ql, double rl) {

	// Compute motor positions
	qmA = ql - acos(rl);
	qmB = ql + acos(rl);
	
	// Return motor positions
	return std::make_tuple(qmA, qmB);
	
}


std::tuple<double, double> ASCCommonToolkit::motorVel2LegVel(double qmA, double qmB, double dqmA, double dqmB) {
	
	// Compute leg velocities
	dql = (dqmA + dqmB)/2.0;
	drl = -(sin((qmA - qmB)/2.0)*(dqmA - dqmB))/2.0;
	
	// Return motor velocities
	return std::make_tuple(dql, drl);
	
}


std::tuple<double, double> ASCCommonToolkit::legVel2MotorVel(double rl, double dql, double drl) {

	// Compute motor velocities
	dqmA = dql + drl/sqrt(1.0 - pow(rl, 2));
	dqmB = dql - drl/sqrt(1.0 - pow(rl, 2));
	
	// Return motor velocities
	return std::make_tuple(dqmA, dqmB);
	
}


double ASCCommonToolkit::rad2Deg(double rad) {

	// Compute degrees
	deg = rad/PI*180.0;
	
	// Return degrees
	return deg;
	
}


double ASCCommonToolkit::deg2Rad(double deg) {

	// Compute radians
	rad = deg/180.0*PI;
	
	// Return radians
	return rad;
	
}


std::tuple<double, double> ASCCommonToolkit::cartPos2PolPos(double x, double z) {

	// Compute polar coordinates
	q = atan2(z, x);
	r = sqrt(pow(x, 2) + pow(z, 2));
	
	// Return polar coordinates
	return std::make_tuple(q, r);
	
}


std::tuple<double, double> ASCCommonToolkit::polPos2CartPos(double q, double r) {

	// Compute polar coordinates
	x = r*cos(q);
	z = r*sin(q);
	
	// Return polar coordinates
	return std::make_tuple(x, z);
	
}


std::tuple<double, double> ASCCommonToolkit::cartVel2PolVel(double q, double r, double dx, double dz) {

	// Compute polar coordinates
	dq = (dz*cos(q) - dx*sin(q))/r;
	dr = dx*cos(q) + dz*sin(q);
	
	// Return polar coordinates
	return std::make_tuple(dq, dr);
	
}


std::tuple<double, double> ASCCommonToolkit::polVel2CartVel(double q, double r, double dq, double dr) {

	// Compute polar coordinates
	dx = dr*cos(q) - r*dq*sin(q);
	dz = dr*sin(q) + r*dq*cos(q);
	
	// Return polar coordinates
	return std::make_tuple(dx, dz);
	
}

}
}
