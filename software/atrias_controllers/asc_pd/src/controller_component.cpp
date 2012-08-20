/*! \file controller_component.cpp
 *  \author Soo-Hyun Yoo
 *  \brief Orocos Component code for the PD subcontroller.
 */

#include <asc_pd/controller_component.h>

namespace atrias {
namespace controller {

ASCPD::ASCPD(std::string name):
    RTT::TaskContext(name),
    P(0.0),
    D(0.0),
    logPort(name + "_log")
{
    this->provides("pd")
        ->addOperation("runController", &ASCPD::runController, this, OwnThread)
        .doc("Run the controller.");

    this->addProperty("P", P)
        .doc("P gain");
    this->addProperty("D", D)
        .doc("D gain");

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

    log(Info) << "[ASCPD] Motor position controller constructed!" << endlog();
}

double ASCPD::runController(double targetPos, double currentPos, double targetVel, double currentVel) {
    // The magic control code
    out = P * (targetPos - currentPos) + D * (targetVel - currentVel);

    // Stuff the msg and push to ROS for logging
    logData.output = out;
    logPort.write(logData);

    return out;
}

bool ASCPD::configureHook() {
    log(Info) << "[ASCPD] configured!" << endlog();
    return true;
}

bool ASCPD::startHook() {
    log(Info) << "[ASCPD] started!" << endlog();
    return true;
}

void ASCPD::updateHook() {
}

void ASCPD::stopHook() {
    log(Info) << "[ASCPD] stopped!" << endlog();
}

void ASCPD::cleanupHook() {
    log(Info) << "[ASCPD] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCPD)

}
}
