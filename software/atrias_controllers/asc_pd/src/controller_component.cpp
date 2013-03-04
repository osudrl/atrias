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
    logPort("logOutput")
{
    this->provides("pd")
        ->addOperation("runController", &ASCPD::runController, this, ClientThread)
        .doc("Run the controller.");

    this->addProperty("P", P)
        .doc("P gain");
    this->addProperty("D", D)
        .doc("D gain");

    // For logging
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

    log(Info) << "[ASCPD] Motor position controller constructed!" << endlog();
}

double ASCPD::runController(double targetPos, double currentPos, double targetVel, double currentVel) {
    // The control code
    out = P * (targetPos - currentPos) + D * (targetVel - currentVel);

    // Stuff the msg and push to ROS for logging
    logData.header = getROSHeader();
    logData.P = P;
    logData.D = D;
    logData.targetPos = targetPos;
    logData.currentPos = currentPos;
    logData.targetVel = targetVel;
    logData.currentVel = currentVel;
    logData.output = out;
    logPort.write(logData);

    return out;
}

bool ASCPD::configureHook() {
    RTT::TaskContext* rtOpsPeer = this->getPeer("Deployer")->getPeer("atrias_rt");
    if (rtOpsPeer) {
        getROSHeader = rtOpsPeer->provides("timestamps")->getOperation("getROSHeader");
    }
    else {
        log(Warning) << "[ASCPD] Can't connect to the Deployer" << endlog();
    }

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
