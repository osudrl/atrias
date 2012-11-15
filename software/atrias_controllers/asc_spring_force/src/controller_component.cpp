/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the asc_spring_force subcontroller.
 */

#include <asc_spring_force/controller_component.h>

namespace atrias {
namespace controller {

ASCSpringForce::ASCSpringForce(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log")
{
    this->provides("exampleService")
        ->addOperation("runController", &ASCSpringForce::runController, this, ClientThread)
        .doc("Run the controller.");

    // Add properties
    this->addProperty("springTorque0Name", springTorque0Name);
    this->addProperty("springTorque1Name", springTorque1Name);

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

    log(Info) << "[ASCSpringForce] Constructed!" << endlog();
}

// Put control code here.
double ASCSpringForce::runController(double exampleInput) {
    out = exampleInput;

    // Stuff the msg and push to ROS for logging
    logData.input = exampleInput;
    logData.output = out;
    logPort.write(logData);

    // Output for the parent controller
    return out;
}

bool ASCSpringForce::configureHook() {
    // Connect to the subcontrollers
    springTorque0 = this->getPeer(springTorque0Name);
    if (springTorque0)
        springTorque0GetTorque = springTorque0->provides("springTorque")->getOperation("getTorque");

    springTorque1 = this->getPeer(springTorque1Name);
    if (springTorque1)
        springTorque1GetTorque = springTorque1->provides("springTorque")->getOperation("getTorque");

    // Get references to subcontroller component properties
    linearInterp0Name0 = springTorque0->properties()->getProperty("linearInterp0Name");
    linearInterp0Name1 = springTorque1->properties()->getProperty("linearInterp0Name");

    log(Info) << "[ASCSpringForce] configured!" << endlog();
    return true;
}

bool ASCSpringForce::startHook() {
    log(Info) << "[ASCSpringForce] started!" << endlog();
    return true;
}

void ASCSpringForce::updateHook() {
}

void ASCSpringForce::stopHook() {
    log(Info) << "[ASCSpringForce] stopped!" << endlog();
}

void ASCSpringForce::cleanupHook() {
    log(Info) << "[ASCSpringForce] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCSpringForce)

}
}
