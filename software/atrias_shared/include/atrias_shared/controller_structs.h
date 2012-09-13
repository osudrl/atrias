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

struct SinOut {
    double pos;
    double vel;
};

struct RobotPosLegHalf {
    double A;
    double B;
};

struct RobotPosLeg {
    double hip;
    RobotPosLegHalf halfA;
    RobotPosLegHalf halfB;
};

struct RobotPos {
    RobotPosLeg lLeg;
    RobotPosLeg rLeg;
};

}
}

#endif
