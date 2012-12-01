/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the asc_toe_decode subcontroller.
 */

#include <asc_toe_decode/controller_component.h>

namespace atrias {
namespace controller {

ASCToeDecode::ASCToeDecode(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log")
{
    this->provides("exampleService")
        ->addOperation("runController", &ASCToeDecode::runController, this, ClientThread)
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

    log(Info) << "[ASCToeDecode] Constructed!" << endlog();
}

// Put control code here.
double ASCToeDecode::runController(double exampleInput) {
    out = exampleInput;

    // Stuff the msg and push to ROS for logging
    logData.input = exampleInput;
    logData.output = out;
    logPort.write(logData);

    // Output for the parent controller
    return out;
}

bool ASCToeDecode::configureHook() {
    log(Info) << "[ASCToeDecode] configured!" << endlog();
    return true;
}

bool ASCToeDecode::startHook() {
    log(Info) << "[ASCToeDecode] started!" << endlog();
    return true;
}

void ASCToeDecode::updateHook() {
}

void ASCToeDecode::stopHook() {
    log(Info) << "[ASCToeDecode] stopped!" << endlog();
}

void ASCToeDecode::cleanupHook() {
    log(Info) << "[ASCToeDecode] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCToeDecode)

}
}
