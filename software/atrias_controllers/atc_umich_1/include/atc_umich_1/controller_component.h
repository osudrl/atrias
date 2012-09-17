#ifndef __ATC_UMICH_1_H__
#define __ATC_UMICH_1_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for atc_umich_1 controller.
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
#include <atrias_shared/globals.h>
#include <robot_invariant_defs.h>

// Datatypes
#include <atc_umich_1/controller_input.h>
#include <atc_umich_1/controller_log_data.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/controller_structs.h>

// MATLAB code
extern "C" {
#include <simulink_name.h>
}

using namespace RTT;
using namespace Orocos;
using namespace atc_umich_1;

namespace atrias {
using namespace shared;
namespace controller {

class ATCUmich1 : public TaskContext {
private:
    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);

    atrias_msgs::controller_output co;

    // For the GUI
    shared::GuiPublishTimer                         *pubTimer;
    controller_input                                guiIn;
    InputPort<controller_input>                     guiDataIn;

    // For logging
    atc_umich_1::controller_log_data logData;
    OutputPort<atc_umich_1::controller_log_data> logPort;


public:
    // Constructor
    ATCUmich1(std::string name);

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
