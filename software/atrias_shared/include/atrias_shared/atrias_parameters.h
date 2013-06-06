// Title: atrias_parameters
// Description: Defines current physical ATRIAS parameters.
// Author: Mikhail Jones

#ifndef ATRIAS_PARAMETERS_H_
#define ATRIAS_PARAMETERS_H_

namespace atrias {
namespace controller {

// ATRIAS control frequency [sec]
#define H 0.001
	
// ATRIAS leg segment lengths [m]
#define L1 0.5
#define L2 0.5

// ATRIAS rotational spring constant [N*m/rad]
#define KS 1895.0

// ATRIAS motor torque constant [N*m/A]
#define KT 0.0987

// ATRIAS leg harmonic drive gear ratio
#define KG 50.0

// ATRIAS mass [kg]
#define M 58.8

// Gravity [m/sec^2]
#define G 9.81

// Pi constant
#define PI 3.14159265

} // namespace controller
} // namespace atrias

#endif
