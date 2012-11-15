/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the asc_torque_defl subcontroller.
 */

#include <asc_torque_defl/controller_component.h>

namespace atrias {
namespace controller {

ASCTorqueDefl::ASCTorqueDefl(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log")
{
    this->provides("exampleService")
        ->addOperation("runController", &ASCTorqueDefl::runController, this, ClientThread)
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

    log(Info) << "[ASCTorqueDefl] Constructed!" << endlog();
}

// Put control code here.
double ASCTorqueDefl::runController(double exampleInput) {
    out = exampleInput;

    // Stuff the msg and push to ROS for logging
    logData.input = exampleInput;
    logData.output = out;
    logPort.write(logData);

    // Output for the parent controller
    return out;
}

bool ASCTorqueDefl::configureHook() {
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

    log(Info) << "[ASCTorqueDefl] configured!" << endlog();
    return true;
}

bool ASCTorqueDefl::startHook() {
    log(Info) << "[ASCTorqueDefl] started!" << endlog();
    return true;
}

void ASCTorqueDefl::updateHook() {
}

void ASCTorqueDefl::stopHook() {
    log(Info) << "[ASCTorqueDefl] stopped!" << endlog();
}

void ASCTorqueDefl::cleanupHook() {
    log(Info) << "[ASCTorqueDefl] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCTorqueDefl)

}
}
