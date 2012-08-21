/*! \file controller_component.h
 *  \author Andrew Peekema
 *  \brief Orocos Component header for atc_template controller.
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

// Datatypes
#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_output.h>
#include <atc_template/controller_input.h>
#include <atrias_shared/controller_structs.h>

using namespace RTT;
using namespace Orocos;
using namespace atc_template;

namespace atrias {
using namespace shared;
namespace controller {

class ATCTemplate : public TaskContext {
private:
    atrias_msgs::robot_state         robotState;
    atrias_msgs::controller_output   controllerOutput;

    controller_input                 guiIn;
    controller_status                guiOut;

    InputPort<controller_input>      guiDataIn;
    OutputPort<controller_status>    guiDataOut;

    // Service plugins
    // Service components 

    // This Operation is called by the RT Operations Manager.
    atrias_msgs::controller_output runController(atrias_msgs::robot_state);

public:
    // Constructor
    ATCTemplate(std::string name);

    // Standard Orocos hooks
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
}
}
