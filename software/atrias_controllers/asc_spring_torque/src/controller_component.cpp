/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the asc_spring_torque subcontroller.
 */

#include <asc_spring_torque/controller_component.h>

namespace atrias {
namespace controller {

ASCSpringTorque::ASCSpringTorque(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log")
{
    this->provides("exampleService")
        ->addOperation("runController", &ASCSpringTorque::runController, this, ClientThread)
        .doc("Run the controller.");

    // Add properties
    this->addProperty("linearInterp0Name", linearInterp0Name);

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

    log(Info) << "[ASCSpringTorque] Constructed!" << endlog();
}

// Put control code here.
double ASCSpringTorque::runController(double exampleInput) {
    out = exampleInput;

    // Stuff the msg and push to ROS for logging
    logData.input = exampleInput;
    logData.output = out;
    logPort.write(logData);

    // Output for the parent controller
    return out;
}

bool ASCSpringTorque::configureHook() {
    // Connect to the subcontrollers
    linearInterp0 = this->getPeer(linearInterp0Name);
    if (linearInterp0)
        linearInterp0InputPoints = linearInterp0->provides("interp")->getOperation("inputPoints");

    linearInterp0 = this->getPeer(linearInterp0Name);
    if (linearInterp0)
        linearInterp0GetValue = linearInterp0->provides("interp")->getOperation("getValue");

    linearInterp0 = this->getPeer(linearInterp0Name);
    if (linearInterp0)
        linearInterp0InputPoints = linearInterp0->provides("interp")->getOperation("inputPoints");

    linearInterp0 = this->getPeer(linearInterp0Name);
    if (linearInterp0)
        linearInterp0GetValue = linearInterp0->provides("interp")->getOperation("getValue");

    // Get references to subcontroller component properties

    log(Info) << "[ASCSpringTorque] configured!" << endlog();
    return true;
}

bool ASCSpringTorque::startHook() {
    log(Info) << "[ASCSpringTorque] started!" << endlog();
    return true;
}

void ASCSpringTorque::updateHook() {
}

void ASCSpringTorque::stopHook() {
    log(Info) << "[ASCSpringTorque] stopped!" << endlog();
}

void ASCSpringTorque::cleanupHook() {
    log(Info) << "[ASCSpringTorque] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCSpringTorque)

}
}
