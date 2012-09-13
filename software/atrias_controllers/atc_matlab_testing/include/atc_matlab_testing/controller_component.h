#ifndef __ATC_MATLAB_TESTING_H__
#define __ATC_MATLAB_TESTING_H__

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
extern "C" {
//#include <open_loop_sin_cos.h>
//#include <open_loop_sin_deriv.h>
//#include <open_loop_sin_cos_6A.h>
//#include <leg_position_pd_test.h>
#include <leg_pos_pd_2.h>
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

    // For the GUI
    controller_input                                guiIn;

    InputPort<controller_input>                     guiDataIn;

    shared::GuiPublishTimer                         *pubTimer;

    // Logging
    controller_log_data              logData;
    OutputPort<controller_log_data>  logPort;

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
