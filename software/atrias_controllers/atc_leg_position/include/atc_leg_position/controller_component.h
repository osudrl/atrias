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

    atrias_msgs::robot_state        robotState;
    atrias_msgs::controller_output  controllerOutput;

    controller_input                guiIn;
    controller_status               guiOut;

    InputPort<controller_input>     guiDataIn;
    OutputPort<controller_status>   guiDataOut;

    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state);

    // Variables for subcontrollers
    std::string pd0Name;
    std::string pd1Name;

    TaskContext *pd0;
    TaskContext *pd1;

    Property<double> P0;
    Property<double> D0;
    Property<double> P1;
    Property<double> D1;

    // Subcontroller operations
    OperationCaller<double(double, double, double, double)> pd0Controller;
    OperationCaller<double(double, double, double, double)> pd1Controller;
    OperationCaller<MotorAngle(double, double)> legToMotorPos;

    // Math variables
    double targetPos, currentPos, targetVel, currentVel;
    MotorAngle motorAngle;

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
