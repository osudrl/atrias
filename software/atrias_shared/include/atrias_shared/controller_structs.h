/*
 * Structs for controller/subcontroller communication
 */

#ifndef __CONTROLLER_STRUCTS_H__
#define __CONTROLLER_STRUCTS_H__

namespace atrias {
namespace controller {

// General AB struct
struct AB {
    double A;
    double B;
};

// General LeftRight struct
struct LeftRight {
    double left;
    double right;
};

// Used in ASCLegForce and ASCSlipModel
struct LegForce {
	double fx;
	double fz;
	double dfx;
	double dfz;
};

// Used in ASCLegForce
struct Gain {
	double kp; // Proportional gain
	double kd; // Derivative gain
	double ks; // Spring constant
	double kg; // Gear ratio
	double kt; // Torque constant
};

// Used in ASCSlipModel
struct SlipState {
	bool isFlight;
	double r; // Leg length
	double dr; // Change in leg length
	double q; // Leg angle
	double dq; // Change in leg angle
	double rOld; // Leg length at (n-1) timestep
	double drOld; // Change in leg length at (n-1) timestep
	double qOld; // Leg angle at (n-1) timestep
	double dqOld; // Change in leg angle at (n-1) timestep
};

// Used in ASCSlipModel
struct SlipModel {
	double g; // Gravity
	double k; // Rotational spring constant
	double m; // Mass
	double r0; // Initial leg length
};

struct MotorAngle {
    double A;
    double B;
};

struct MotorVelocity {
    double A;
    double B;
};

struct MotorState {
    double ang;
    double vel;
};

struct RobotPosLeg {
    double hip;
    double A;
    double B;
};

struct LegState {
	MotorState A;
	MotorState B;
};

struct RobotPos {
    RobotPosLeg lLeg;
    RobotPosLeg rLeg;
};

struct RobotLeg {
    double ang;
    double angVel;
    double len;
    double lenVel;
};

struct RobotSide {
    RobotLeg leg;
    MotorState hip;
};

struct DesiredRobotState {
    RobotSide left;
    RobotSide right;
};

}
}

#endif
