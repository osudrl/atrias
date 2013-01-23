/*
 * Structs for controller/subcontroller communication
 */

#ifndef __CONTROLLER_STRUCTS_H__
#define __CONTROLLER_STRUCTS_H__

namespace atrias {
namespace controller {

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
