#ifndef __ATC_VELOCITY_TUNING__
#define __ATC_VELOCITY_TUNING__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for atc_velocity_tuning controller.
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
#include <atc_velocity_tuning/controller_input.h>
#include <atc_velocity_tuning/controller_status.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_velocity_tuning;

namespace atrias {
using namespace shared;
namespace controller {

class ATCVelocityTuning : public TaskContext {
private:
    shared::GuiPublishTimer         *pubTimer;

    atrias_msgs::robot_state         robotState;
    atrias_msgs::controller_output   controllerOutput;

    controller_input                 guiIn;
    controller_status                guiOut;

    OutputPort<controller_status>    guiDataOut;
    InputPort<controller_input>      guiDataIn;

    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state);

public:
    // Constructor
    ATCVelocityTuning(std::string name);

    // Standard Orocos hooks
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
}
}

#endif
