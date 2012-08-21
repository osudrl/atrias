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

// Datatypes
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atc_motor_sin_wave/controller_input.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_motor_sin_wave;

namespace atrias {
using namespace shared;
namespace controller {

class ATCMotorSinWave : public TaskContext {
private:
    atrias_msgs::robot_state        robotState;
    atrias_msgs::controller_output  controllerOutput;

    controller_input                guiIn;

    InputPort<controller_input>     guiDataIn;

    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state);

    // Variables for subcontrollers
    std::string pd0Name;
    std::string sin0Name;

    TaskContext *pd0;
    TaskContext *sin0;

    Property<double> P0;
    Property<double> D0;

    OperationCaller<SinOut(double, double)> sin0Controller;
    OperationCaller<double(double, double, double, double)> pd0Controller;

    // Math variables
    double centerBAngle;
    double targetPos, currentPos, targetVel, currentVel;
    SinOut motorBSin;


public:
    // Constructor
    ATCMotorSinWave(std::string name);

    // Standard Orocos hooks
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
}
}