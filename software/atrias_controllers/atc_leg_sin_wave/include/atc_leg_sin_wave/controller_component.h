/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for leg sin wave controller.
 */

// Orocos 
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Component.hpp>
#include <rtt/Property.hpp>

// C
#include <stdlib.h>

#include <atrias_shared/GuiPublishTimer.h>
#include <atrias_shared/globals.h>
#include <robot_invariant_defs.h>

// Datatypes
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atc_leg_sin_wave/controller_input.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_leg_sin_wave;

namespace atrias {
using namespace shared;
namespace controller {

class ATCLegSinWave : public TaskContext {
private:
    atrias_msgs::controller_output  co;

    controller_input                guiIn;

    InputPort<controller_input>     guiDataIn;

    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state);

    // Variables for subcontrollers
    std::string pd0Name;
    std::string pd1Name;
    std::string pd2Name;
    std::string pd3Name;
    std::string pd4Name;
    std::string pd5Name;
    std::string sin0Name;
    std::string sin1Name;
    std::string sin2Name;
    std::string sin3Name;
    std::string sin4Name;
    std::string sin5Name;

    TaskContext *pd0;
    TaskContext *pd1;
    TaskContext *pd2;
    TaskContext *pd3;
    TaskContext *pd4;
    TaskContext *pd5;
    TaskContext *sin0;
    TaskContext *sin1;
    TaskContext *sin2;
    TaskContext *sin3;
    TaskContext *sin4;
    TaskContext *sin5;

    Property<double> P0;
    Property<double> D0;
    Property<double> P1;
    Property<double> D1;
    Property<double> P2;
    Property<double> D2;
    Property<double> P3;
    Property<double> D3;
    Property<double> P4;
    Property<double> D4;
    Property<double> P5;
    Property<double> D5;

    OperationCaller<MotorState(double, double)> sin0Controller;
    OperationCaller<MotorState(double, double)> sin1Controller;
    OperationCaller<MotorState(double, double)> sin2Controller;
    OperationCaller<MotorState(double, double)> sin3Controller;
    OperationCaller<MotorState(double, double)> sin4Controller;
    OperationCaller<void(double)>               sin4SetPhase;
    OperationCaller<MotorState(double, double)> sin5Controller;
    OperationCaller<void(double)>               sin5SetPhase;
    OperationCaller<MotorAngle(double, double)> legToMotorPos;
    OperationCaller<MotorVelocity(MotorState, MotorState)> legToMotorVel;
    OperationCaller<double(double, double, double, double)> pd0Controller;
    OperationCaller<double(double, double, double, double)> pd1Controller;
    OperationCaller<double(double, double, double, double)> pd2Controller;
    OperationCaller<double(double, double, double, double)> pd3Controller;
    OperationCaller<double(double, double, double, double)> pd4Controller;
    OperationCaller<double(double, double, double, double)> pd5Controller;

    // Math variables
    double centerLength, centerAngle, hipCenter;
    double targetPos, currentPos, targetVel, currentVel;
    double hipPeriod, vertical;
    MotorAngle lMotorAngle, rMotorAngle;
    MotorVelocity lMotorVelocity, rMotorVelocity;
    MotorState lLegLen, lLegAng, rLegLen, rLegAng, lHip, rHip;

    // Debugging
    int count;


public:
    // Constructor
    ATCLegSinWave(std::string name);

    // Standard Orocos hooks
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
}
}
