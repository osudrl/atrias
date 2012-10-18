/*! \file controller_component.h
 *  \author Soo-Hyun Yoo
 *  \brief Orocos Component header for leg position controller.
 */

// Orocos 
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Component.hpp>
#include <rtt/Property.hpp>

// C
#include <stdlib.h>

#include <atrias_shared/GuiPublishTimer.h>
#include <atrias_shared/globals.h>
#include <robot_invariant_defs.h>

// Datatypes
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atc_leg_position/controller_input.h>
#include <atc_leg_position/controller_status.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_leg_position;

namespace atrias {
using namespace shared;
namespace controller {

class ATCLegPosition : public TaskContext {
private:
    shared::GuiPublishTimer         *pubTimer;

    atrias_msgs::robot_state        rs;
    atrias_msgs::controller_output  co;

    controller_input                guiIn;
    controller_status               guiOut;

    InputPort<controller_input>     guiDataIn;
    OutputPort<controller_status>   guiDataOut;

    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state);

    // Variables for subcontrollers
    std::string robotPd0Name;

    TaskContext *robotPd0;

    Property<double> legP;
    Property<double> legD;
    Property<double> hipP;
    Property<double> hipD;

    // Subcontroller operations
    OperationCaller<atrias_msgs::controller_output(atrias_msgs::robot_state, DesiredRobotState)> robotPd0Controller;

    // Math variables
    DesiredRobotState ds;

public:
    // Constructor
    ATCLegPosition(std::string name);

    // Standard Orocos hooks
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
}
}
