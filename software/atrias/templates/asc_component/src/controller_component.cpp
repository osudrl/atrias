/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the asc_component subcontroller.
 */

#include <asc_component/controller_component.h>

namespace atrias {
namespace controller {

ASCComponent::ASCComponent(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log")
{
    this->provides("exampleService")
        ->addOperation("runController", &ASCComponent::runController, this, OwnThread)
        .doc("Run the controller.");

    // Add properties

    // Logging
    // Create a port
    addPort(logPort); 
    // Unbuffered
    ConnPolicy policy = RTT::ConnPolicy::data();
    // Transport type = ROS
    policy.transport = 3;
    // ROS topic name
    policy.name_id = "/" + name + "_log";
    // Construct the stream between the port and ROS topic
    logPort.createStream(policy);

    log(Info) << "[ASCComponent] asc_component controller constructed!" << endlog();
}

// Put control code here.
double ASCComponent::runController(double exampleInput) {
    out = exampleInput;

    // Stuff the msg and push to ROS for logging
    logData.input = exampleInput;
    logData.output = out;
    logPort.write(logData);

    // Output for the parent controller
    return out;
}

bool ASCComponent::configureHook() {
    // Connect to the subcontrollers
    // Service plugins

    // Get references to subcontroller component properties

    log(Info) << "[ASCComponent] configured!" << endlog();
    return true;
}

bool ASCComponent::startHook() {
    log(Info) << "[ASCComponent] started!" << endlog();
    return true;
}

void ASCComponent::updateHook() {
}

void ASCComponent::stopHook() {
    log(Info) << "[ASCComponent] stopped!" << endlog();
}

void ASCComponent::cleanupHook() {
    log(Info) << "[ASCComponent] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCComponent)

}
}
