/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the asc_linear_interp subcontroller.
 */

#include <asc_linear_interp/controller_component.h>

namespace atrias {
namespace controller {

ASCLinearInterp::ASCLinearInterp(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log")
{
    this->provides("exampleService")
        ->addOperation("runController", &ASCLinearInterp::runController, this, ClientThread)
        .doc("Run the controller.");

    // Logging
    // Create a port
    addPort(logPort); 
    // Connect with buffer size 100000 so we get all data.
    ConnPolicy policy = RTT::ConnPolicy::buffer(100000);
    // Transport type = ROS
    policy.transport = 3;
    // ROS topic name
    policy.name_id = "/" + name + "_log";
    // Construct the stream between the port and ROS topic
    logPort.createStream(policy);

    log(Info) << "[ASCLinearInterp] Constructed!" << endlog();
}

// Put control code here.
double ASCLinearInterp::runController(double exampleInput) {
    out = exampleInput;

    // Stuff the msg and push to ROS for logging
    logData.input = exampleInput;
    logData.output = out;
    logPort.write(logData);

    // Output for the parent controller
    return out;
}

bool ASCLinearInterp::configureHook() {
    log(Info) << "[ASCLinearInterp] configured!" << endlog();
    return true;
}

bool ASCLinearInterp::startHook() {
    log(Info) << "[ASCLinearInterp] started!" << endlog();
    return true;
}

void ASCLinearInterp::updateHook() {
}

void ASCLinearInterp::stopHook() {
    log(Info) << "[ASCLinearInterp] stopped!" << endlog();
}

void ASCLinearInterp::cleanupHook() {
    log(Info) << "[ASCLinearInterp] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCLinearInterp)

}
}
