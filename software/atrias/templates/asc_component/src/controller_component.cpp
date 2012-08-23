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
    this->provides("pd")
        ->addOperation("runController", &ASCComponent::runController, this, OwnThread)
        .doc("Run the controller.");

    // For logging
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

    log(Info) << "[ASCComponent] Motor position controller constructed!" << endlog();
}

double ASCComponent::runController(double exampleInput) {
    // The magic control code
    out = exampleInput;

    // Stuff the msg and push to ROS for logging
    logData.output = out;
    logPort.write(logData);

    return out;
}

bool ASCComponent::configureHook() {
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
