#ifndef __ATC_DEMO_RANGE_OF_MOTION_H__
#define __ATC_DEMO_RANGE_OF_MOTION_H__

/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for atc_demo_range_of_motion controller.
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
#include <atc_demo_range_of_motion/controller_input.h>
#include <atc_demo_range_of_motion/controller_status.h>
#include <atc_demo_range_of_motion/controller_log_data.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_demo_range_of_motion;

namespace atrias {
using namespace shared;
namespace controller {

class ATCDemoRangeOfMotion : public TaskContext {
private:
    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);

    atrias_msgs::controller_output co;

    // Subcontroller names
    std::string pd0Name;
    std::string pd1Name;
    std::string pd2Name;
    std::string pd3Name;
    std::string pd4Name;
    std::string pd5Name;
    std::string spg0Name;
    std::string spg1Name;
    std::string spg2Name;
    std::string spg3Name;
    std::string spg4Name;
    std::string spg5Name;

    // Subcontroller components
    TaskContext *pd0;
    TaskContext *pd1;
    TaskContext *pd2;
    TaskContext *pd3;
    TaskContext *pd4;
    TaskContext *pd5; 
    TaskContext *spg0;
    TaskContext *spg1;
    TaskContext *spg2;
    TaskContext *spg3;
    TaskContext *spg4;
    TaskContext *spg5;

    // Service properties
    Property<double> D0;
    Property<double> D1;
    Property<double> D2;
    Property<double> D3;
    Property<double> D4;
    Property<double> D5;
    Property<double> P0;
    Property<double> P1;
    Property<double> P2;
    Property<double> P3;
    Property<double> P4;
    Property<double> P5;

    // Subcontroller operations
    OperationCaller<double(double, double, double, double)> pd0RunController;
    OperationCaller<double(double, double, double, double)> pd1RunController;
    OperationCaller<double(double, double, double, double)> pd2RunController;
    OperationCaller<double(double, double, double, double)> pd3RunController;
    OperationCaller<double(double, double, double, double)> pd4RunController;
    OperationCaller<double(double, double, double, double)> pd5RunController;
    OperationCaller<void(double, double, double)> spg0Init;
    OperationCaller<void(double, double, double)> spg1Init;
    OperationCaller<void(double, double, double)> spg2Init;
    OperationCaller<void(double, double, double)> spg3Init;
    OperationCaller<void(double, double, double)> spg4Init;
    OperationCaller<void(double, double, double)> spg5Init;
    OperationCaller<MotorState(void)> spg0RunController;
    OperationCaller<MotorState(void)> spg1RunController;
    OperationCaller<MotorState(void)> spg2RunController;
    OperationCaller<MotorState(void)> spg3RunController;
    OperationCaller<MotorState(void)> spg4RunController;
    OperationCaller<MotorState(void)> spg5RunController;

    // Current desired motor states
    MotorState desLeftAState,
               desLeftBState,
               desLeftHipState,
               desRightAState,
               desRightBState,
               desRightHipState;

    // Logging
    controller_log_data              logData;
    OutputPort<controller_log_data>  logPort;

    // For the GUI
    shared::GuiPublishTimer                         *pubTimer;
    controller_input                                guiIn;
    InputPort<controller_input>                     guiDataIn;

public:
    // Constructor
    ATCDemoRangeOfMotion(std::string name);

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
