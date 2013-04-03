// Title: atrias_parameters
// Description: Defines current physical ATRIAS parameters.
// Author: Mikhail Jones

#ifndef ATRIAS_PARAMETERS_H_
#define ATRIAS_PARAMETERS_H_

namespace atrias {
namespace controller {

// ATRIAS control frequency [sec]
double h = 0.001;
	
// ATRIAS leg segment lengths [m]
double l1 = 0.5;
double l2 = 0.5;

// ATRIAS rotational spring constant [N*m/rad]
double ks = 4118.0;

// ATRIAS motor torque constant [N*m/A]
double kt = 0.0987;

// ATRIAS leg harmonic drive gear ratio
double kg = 50.0;

// ATRIAS mass [kg]
double m = 61.93;

// Gravity [m/sec^2]
double g = 9.81;

// Pi constant
double PI = 3.14159265;

} // namespace controller
} // namespace atrias

#endif
