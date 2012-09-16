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

struct SinOut {
    double pos;
    double vel;
};

struct RobotPosLeg {
    double hip;
    double A;
    double B;
};

struct RobotPos {
    RobotPosLeg lLeg;
    RobotPosLeg rLeg;
};

}
}

#endif
