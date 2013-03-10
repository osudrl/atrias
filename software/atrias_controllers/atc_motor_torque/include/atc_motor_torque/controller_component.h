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

// Datatypes
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atc_motor_torque/controller_input.h>

#define AMC_IP 1.0
#define AMC_IC 0.5
#define AMC_PEAK_TIME 2.0
#define AMC_FOLDBACK_TIME 10.0

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
    float curLimit, td;
    bool foldbackTriggered;

    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);

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
