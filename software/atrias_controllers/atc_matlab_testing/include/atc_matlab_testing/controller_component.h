#ifndef __ATC_COMPONENT_H__
#define __ATC_COMPONENT_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for atc_matlab_testing controller.
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

// Datatypes
#include <atc_matlab_testing/controller_input.h>
#include <atc_matlab_testing/controller_status.h>
#include <atc_matlab_testing/controller_log_data.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/controller_structs.h>

// MATLAB
//#define MODEL open_loop_sin_cos
//#define MODEL open_loop_sin_deriv
#define MODEL open_loop_sin_cos_6A
#define NUMST 2
#define NCSTATES 0
#define HAVESTDIO
#define UNIX
#define ONESTEPFCN 1
#define TERMFCN 1
#define MAT_FILE 0
#define MULTI_INSTANCE_CODE 0
#define INTEGER_CODE 0
#define MT 0
#define TID01EQ 1
extern "C" {
//#include <open_loop_sin_cos.h>
#include <open_loop_sin_deriv.h>
//#include <open_loop_sin_cos_6A.h>
}

using namespace RTT;
using namespace Orocos;
using namespace atc_matlab_testing;

namespace atrias {
using namespace shared;
namespace controller {

class ATCMatlabTesting : public TaskContext {
private:
    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);

    atrias_msgs::controller_output co;

    // Logging
    controller_log_data              logData;
    OutputPort<controller_log_data>  logPort;

    // For the GUI
    shared::GuiPublishTimer                         *pubTimer;
    controller_input                                guiIn;
    InputPort<controller_input>                     guiDataIn;

public:
    // Constructor
    ATCMatlabTesting(std::string name);

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
