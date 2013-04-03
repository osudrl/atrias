/**
  * @file ASC_COMMON_TOOLKIT.cpp
  * @author Mikhail Jones
  * @brief This implements common functionality required by most controllers.
  */

// To use do something like this.
// std::tie(qmA, qmB) = polLegPos2MotorPos(ql, rl);

// TODO Finish documenting all functions
// TODO cartLegPos2MotorPos
// TODO cartLegVel2MotorVel
// TODO inverse of all conversions
// TODO interp

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
	// No init required for this controller
}

// legStiffness
double ASCCommonToolkit::legStiffness(double r, double r0) {

	// Compute non-linear ATRIAS virtual leg length stiffness
	k = ks*(sin(acos(r)) - (acos(r) - acos(r0))*cos(acos(r)))/(2.0*l1*l2*pow(sin(acos(r)), 3));
	
    // Set the log data
    log_out.data.k = k;

    // Transmit the log data
    log_out.send();

	return k;
	
} // legStiffness


// polMotorPos2LegPos
std::tuple<double, double> ASCCommonToolkit::polMotorPos2LegPos(double qmA, double qmB) {

	// Compute leg positions
    ql = ((qmA + qmB)/2.0);
    rl = cos((qmA - qmB)/2.0);
	
	return std::make_tuple(ql, rl);
	
} // polMotorPos2LegPos


// polLegPos2MotorPos
std::tuple<double, double> ASCCommonToolkit::polLegPos2MotorPos(double ql, double rl) {

	// Compute motor positions
    qmA = ql - acos(rl);
    qmB = ql + acos(rl);
	
	return std::make_tuple(qmA, qmB);
	
} // polLegPos2MotorPos


// polLegVel2MotorVel
std::tuple<double, double> ASCCommonToolkit::polLegVel2MotorVel(double ql, double dql, double drl) {

	// Compute motor velocities
    dqmA = dql + drl/sqrt(1.0 - pow(ql, 2));
    dqmB = dql - drl/sqrt(1.0 - pow(ql, 2));
	
	return std::make_tuple(dqmA, dqmB);
	
} // polLegVel2MotorVel

// rad2deg
double ASCCommonToolkit::rad2deg(double rad) {

	// Compute motor velocities
    deg = rad/PI*180.0;
	
	// Return computed degrees
	return deg;
	
} // rad2deg

// deg2rad
double ASCCommonToolkit::deg2rad(double deg) {

	// Compute motor velocities
    rad = deg/180.0*PI;
	
	// Return computed radians
	return rad;
	
} // deg2rad

}
}

