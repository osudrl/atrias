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
#include <atrias_shared/globals.h>

// ATRIAS
#include <atrias_shared/drl_math.h>

// Datatypes
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atc_motor_torque/controller_input.h>
#include <asc_slip_model/controller_component.h>

const double AMC_IP = 60.0;   // In amps
const double AMC_IC = 30.0;   // In amps
const double AMC_PEAK_TIME = 2.0;   // In seconds
const double AMC_FOLDBACK_TIME = 10.0;   // In seconds

const double COUNTER_MAX = (AMC_IC + (AMC_IP-AMC_IC) * (AMC_PEAK_TIME+AMC_FOLDBACK_TIME) / AMC_FOLDBACK_TIME);   // In amps
const double M_FB = (COUNTER_MAX-AMC_IC) / (AMC_PEAK_TIME+AMC_FOLDBACK_TIME);   // Foldback slope in amps/second

// ASCSlipModel
const double l1 = 0.5;
const double l2 = 0.5;
const double bodyPitch = 3*M_PI/2.0;
const double kg = 50;
const double kt = 0.0987;

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

    // ASCSlipModel
    std::string ascSlipModel0Name;
    TaskContext *ascSlipModel0;
    OperationCaller<SlipState(SlipModel, SlipState)> slipAdvance0;
    OperationCaller<LegForce(SlipModel, SlipState)> slipForce0;

    SlipModel slipModel;
    SlipState slipState;

    // Duty cycle tester variables
    uint32_t dcCounter;
    uint32_t dcStanceEndTime;
    bool dcInStance;

    // Current limiter variables
    double curCounter, curLimit;   // In amps
    double fbCounter;   // In seconds
    bool inFoldback;

    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state rs);

    // Estimate current limit of AMC amplifiers. Run this every loop.
    void estimateCurrentLimit(atrias_msgs::robot_state rs);

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
