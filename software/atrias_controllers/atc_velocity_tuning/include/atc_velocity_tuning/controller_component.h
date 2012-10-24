#ifndef __ATC_VELOCITY_TUNING_H__
#define __ATC_VELOCITY_TUNING_H__

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
#include <atrias_shared/globals.h>
#include <robot_invariant_defs.h>

// Datatypes
#include <atc_velocity_tuning/controller_input.h>
#include <atc_velocity_tuning/controller_status.h>
#include <atc_velocity_tuning/controller_log_data.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_velocity_tuning;

namespace atrias {
using namespace shared;
namespace controller {

class ATCVelocityTuning : public TaskContext {
private:
    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);

    atrias_msgs::controller_output co;

    // Subcontroller names
    std::string pd0Name;

    // Subcontroller components
    TaskContext *pd0; 

    // Service properties
    Property<double> D0;
    Property<double> P0;

    // Subcontroller operations
    OperationCaller<double(double, double, double, double)> pd0RunController;

    // Logging
    controller_log_data              logData;
    OutputPort<controller_log_data>  logPort;

    // For the GUI
    shared::GuiPublishTimer                         *pubTimer;
    controller_input                                guiIn;
    InputPort<controller_input>                     guiDataIn;

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
