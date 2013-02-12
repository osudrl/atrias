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

// Used in ASCLegForce and ASCSlipModelSolver
struct LegForce {
	double fx;
	double fz;
	double dfx;
	double dfz;
};

// Used in ASCLegForce
struct Gain {
	double kp;
	double kd;
	double ks;
	double kg;
	double kt;
};

// Used in ASCSlipModleSolver
struct SlipConditions {
	double rOld;
	double drOld;
	double qOld;
	double dqOld;
	double r;
	double dr;
	double q;
	double dq;
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
