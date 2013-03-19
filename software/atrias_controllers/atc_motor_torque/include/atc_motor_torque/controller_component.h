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

#include <atrias_shared/GuiPublishTimer.h>
#include <robot_invariant_defs.h>

// ATRIAS
#include <atrias_shared/drl_math.h>

// Datatypes
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atc_motor_torque/controller_input.h>

const double AMC_IP = 1.0;
const double AMC_IC = 0.5;
const double AMC_PEAK_TIME = 4.0;   // In seconds
const double AMC_FOLDBACK_TIME = 2.0;   // In seconds

const double COUNTER_MAX = (AMC_IC + (AMC_IP-AMC_IC) * (AMC_PEAK_TIME+AMC_FOLDBACK_TIME) / AMC_FOLDBACK_TIME);
const double M_FB = (0.001 * (COUNTER_MAX-AMC_IC) / (AMC_PEAK_TIME+AMC_FOLDBACK_TIME));   // Foldback slope.

using namespace RTT;
using namespace Orocos;
using namespace atc_motor_torque;

namespace atrias {
using namespace shared;
namespace controller {

class ATCMotorTorque : public TaskContext {
private:
    atrias_msgs::controller_output   co;

    controller_input                 guiIn;

    InputPort<controller_input>      guiDataIn;

    // Current limiter variables
    double curCounter, curLimit;   // In amps
    double timeSinceFB;   // In seconds
    bool inFoldback;

    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);

    // Estimate current limit of AMC amplifiers. Run this every loop.
    void estimateCurrentLimit();

public:
    // Constructor
    ATCMotorTorque(std::string name);

    // Standard Orocos hooks
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
}
}
