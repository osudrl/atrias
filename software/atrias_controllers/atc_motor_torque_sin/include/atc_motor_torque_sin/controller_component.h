/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for the atc_motor_torque_sin controller.
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
#include <atc_motor_torque_sin/controller_input.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_motor_torque_sin;

namespace atrias {
using namespace shared;
namespace controller {

class ATCMotorTorqueSin : public TaskContext {
private:
    atrias_msgs::controller_output  co;

    controller_input                guiIn;

    InputPort<controller_input>     guiDataIn;


    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state);

    // Variables for subcontrollers
    std::string sin0Name;

    TaskContext *sin0;

    OperationCaller<SinOut(double, double)> sin0Controller;

    // Math variables
    SinOut sinOut;
    int prevMotor;
    double current;


public:
    // Constructor
    ATCMotorTorqueSin(std::string name);

    // Standard Orocos hooks
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
}
}
