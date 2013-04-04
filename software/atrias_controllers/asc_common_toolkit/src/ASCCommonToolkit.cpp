/**
  * @file ASC_COMMON_TOOLKIT.cpp
  * @author Mikhail Jones
  * @brief This implements common functionality required by most controllers.
  */

// To use do something like this.
// k = ASCCommonToolkit.legStiffness(r, r0);
// std::tie(ql, rl) = ASCCommonToolkit.motorPos2LegPos(qmA, qmB);
// std::tie(qmA, qmB) = ASCCommonToolkit.motorPos2LegPos(ql, rl);

#include "asc_common_toolkit/ASCCommonToolkit.hpp"

// Again, we need to put our code inside the appropriate namespaces.
namespace atrias {
namespace controller {

/* Breakdown of the next few lines:
 * We need to call our parent class's constructor,
 * then we can call the LogPort's constructor. The second parameter
 * is the name for this log port, which controls how it appears in the
 * bagfiles.
 */
ASCCommonToolkit::ASCCommonToolkit(AtriasController *parent, string name) :
        AtriasController(parent, name),
        log_out(this, "log")
{
	// Initialize
	ks = KS;
}

// legStiffness
double ASCCommonToolkit::legStiffness(double r, double r0) {

	// Compute non-linear ATRIAS virtual leg length stiffness
	k = ks*(sin(acos(r)) - (acos(r) - acos(r0))*cos(acos(r)))/(2.0*L1*L2*pow(sin(acos(r)), 3));
	
    // Set the log data
    log_out.data.k = k;

    // Transmit the log data
    log_out.send();

	// Return virtual leg stiffness
	return k;
	
} // legStiffness

// motorPos2LegPos
std::tuple<double, double> ASCCommonToolkit::motorPos2LegPos(double qmA, double qmB) {

	// Compute leg positions
    ql = ((qmA + qmB)/2.0);
    rl = cos((qmA - qmB)/2.0);
	
	// Return leg position
	return std::make_tuple(ql, rl);
	
} // motorPos2LegPos

// legPos2MotorPos
std::tuple<double, double> ASCCommonToolkit::legPos2MotorPos(double ql, double rl) {

	// Compute motor positions
    qmA = ql - acos(rl);
    qmB = ql + acos(rl);
	
	// Return motor positions
	return std::make_tuple(qmA, qmB);
	
} // legPos2MotorPos

// motorVel2LegVel
std::tuple<double, double> ASCCommonToolkit::motorVel2legVel(double qmA, double qmB, double dqmA, double dqmB) {
	
	// Compute leg velocities
	dql = (dqmA + dqmB)/2.0;
	drl = -(sin((qmA - qmB)/2.0)*(dqmA - dqmB))/2.0;
	
	// Return motor velocities
	return std::make_tuple(dql, drl);
	
} // legVel2MotorVel

// legVel2MotorVel
std::tuple<double, double> ASCCommonToolkit::legVel2MotorVel(double ql, double dql, double drl) {

	// Compute motor velocities
    dqmA = dql + drl/sqrt(1.0 - pow(ql, 2));
    dqmB = dql - drl/sqrt(1.0 - pow(ql, 2));
	
	// Return motor velocities
	return std::make_tuple(dqmA, dqmB);
	
} // legVel2MotorVel

// rad2deg
double ASCCommonToolkit::rad2deg(double rad) {

	// Compute degrees
    deg = rad/PI*180.0;
	
	// Return degrees
	return deg;
	
} // rad2deg

// deg2rad
double ASCCommonToolkit::deg2rad(double deg) {

	// Compute radians
    rad = deg/180.0*PI;
	
	// Return radians
	return rad;
	
} // deg2rad

// cart2pol
std::tuple<double, double> ASCCommonToolkit::cart2pol(double x, double z) {

	// Compute polar coordinates
    q = atan2(z, x);
    r = sqrt(pow(x, 2) + pow(z, 2));
	
	// Return polar coordinates
	return std::make_tuple(q, r);
	
} // cart2pol

// pol2cart
std::tuple<double, double> ASCCommonToolkit::pol2cart(double q, double r) {

	// Compute polar coordinates
    x = r*cos(q);
    z = r*sin(q);
	
	// Return polar coordinates
	return std::make_tuple(x, z);
	
} // pol2cart

}
}

