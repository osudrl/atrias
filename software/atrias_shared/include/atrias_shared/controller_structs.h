/*
 * Structs for controller/subcontroller communication
 */

#ifndef __CONTROLLER_STRUCTS_H__
#define __CONTROLLER_STRUCTS_H__

namespace atrias {
namespace controller {

// General structures ----------------------------------------------------------
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


// Controller specific structures ----------------------------------------------
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
	double ki; // Integral gain
	double kd; // Derivative gain
	double ks; // Spring constant // TODO REMOVE
	double kg; // Gear ratio // TODO REMOVE
	double kt; // Torque constant // TODO REMOVE
};

// Used in ASCSlipModel
struct SlipState {
	bool isFlight;
	bool isStance;
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
	double g; // Gravity // TODO REMOVE
	double k; // Spring constant
	double m; // Mass
	double r0; // Initial leg length
};


// Robot state structures ------------------------------------------------------
struct MotorAngle {
    double A;
    double B;
};

struct MotorVelocity {
    double A;
    double B;
};

struct MotorState { // TODO REMOVE - People should just use robot state struct
    double ang;
    double vel;
};

struct RobotPosLeg { // TODO REMOVE - People should just use robot state struct
    double hip;
    double A;
    double B;
};

struct LegState { // TODO REMOVE - People should just use robot state struct
	MotorState A;
	MotorState B;
};

struct RobotPos { // TODO REMOVE - People should just use robot state struct
    RobotPosLeg lLeg;
    RobotPosLeg rLeg;
};

struct RobotLeg { // TODO REMOVE - People should just use robot state struct
    double ang;
    double angVel;
    double len;
    double lenVel;
};

struct RobotSide { // TODO REMOVE - People should just use robot state struct
    RobotLeg leg;
    MotorState hip;
};

struct DesiredRobotState { // TODO REMOVE - People should just use robot state struct
    RobotSide left;
    RobotSide right;
};

}
}

#endif
