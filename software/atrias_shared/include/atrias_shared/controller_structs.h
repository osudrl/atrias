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

// Used in ASCSlipModel
struct SlipState {
	bool isFlight;
	bool isStance;
	double r; // Leg length
	double dr; // Change in leg length
	double q; // Leg angle
	double dq; // Change in leg angle
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
