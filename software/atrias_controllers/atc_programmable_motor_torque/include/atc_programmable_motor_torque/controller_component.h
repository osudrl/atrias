/*! \file controller_component.h
 *  \author Soo-Hyun Yoo
 *  \brief Orocos Component header for motor torque controller.
 */

// Orocos 
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Component.hpp>

// C
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <atrias_shared/GuiPublishTimer.h>
#include <robot_invariant_defs.h>

// Datatypes
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/globals.h>
#include <atc_programmable_motor_torque/controller_input.h>

#define CURRENT_ADJUSTMENT_THRESHOLD 1.0

using namespace RTT;
using namespace Orocos;
using namespace atc_programmable_motor_torque;

namespace atrias {
using namespace shared;
namespace controller {

class ATCProgrammableMotorTorque : public TaskContext {
private:
    atrias_msgs::robot_state         robotState;
    atrias_msgs::controller_output   controllerOutput;

    uint32_t                         currentSampleIndex;
    std::vector<double>             configData;

    controller_input                 guiIn;

    InputPort<controller_input>      guiDataIn;

    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state);

public:
    // Constructor
    ATCProgrammableMotorTorque(std::string name);

    // Standard Orocos hooks
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
}
}
