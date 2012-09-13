#ifndef __ATC_BIPED_BOOM_INIT_TEST_H__
#define __ATC_BIPED_BOOM_INIT_TEST_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for atc_biped_boom_init_test controller.
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
#include <atc_biped_boom_init_test/controller_input.h>
#include <atc_biped_boom_init_test/controller_status.h>
#include <atc_biped_boom_init_test/controller_log_data.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_biped_boom_init_test;

namespace atrias {
using namespace shared;
namespace controller {

class ATCBipedBoomInitTest : public TaskContext {
private:
    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);

    atrias_msgs::controller_output co;

    // Subcontroller names
    std::string bipedBoomInit0Name;

    // Subcontroller components
    TaskContext *bipedBoomInit0; 

    // Subcontroller operations
    OperationCaller<MotorCurrent(double, double)> bipedBoomInit0LeftLeg;
    OperationCaller<int(void)> bipedBoomInit0IsInitialized;
    OperationCaller<void(atrias_msgs::robot_state _rs)> bipedBoomInit0PassRobotState;

    // Math variables
    MotorCurrent lMotorCurrent;

    // Logging
    controller_log_data              logData;
    OutputPort<controller_log_data>  logPort;

    // For the GUI
    shared::GuiPublishTimer                         *pubTimer;
    controller_input                                guiIn;
    InputPort<controller_input>                     guiDataIn;

public:
    // Constructor
    ATCBipedBoomInitTest(std::string name);

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
