/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the smooth path generator subcontroller.
 */

#include <asc_path_generator/smooth_path_generator.h>

namespace atrias {
namespace controller {

ASCSmoothPathGenerator::ASCSmoothPathGenerator(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log")
{
    this->provides("smoothPath")
        ->addOperation("run", &ASCSmoothPathGenerator::run, this, ClientThread)
        .doc("Run the controller.");
    this->provides("smoothPath")
        ->addOperation("init", &ASCSmoothPathGenerator::init, this, ClientThread);

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

    // Setup the controller
    accumulator = 0.0;

    log(Info) << name << " Constructed!" << endlog();
}

void ASCSmoothPathGenerator::init(double startAng, double endAng)
{
    // Nothing yet
    return;
}

MotorState ASCSmoothPathGenerator::run(double frequency, double amplitude)
{
    // The function
    accumulator += 0.001*2.0*M_PI*frequency;
    if (accumulator >= 2.0*M_PI)
        accumulator -= 2.0*M_PI;

    // Return the function value and its derivative
    // TODO: Make this equation the proper one
    output.ang = amplitude*sin(accumulator);
    output.vel = amplitude*cos(accumulator)*2.0*M_PI*frequency;

    // Stuff the msg and push to ROS for logging
    logData.ang = output.ang;
    logData.vel = output.vel;
    logPort.write(logData);

    return output;
}

bool ASCSmoothPathGenerator::configureHook() {
    log(Info) << "[ASCSG] configured!" << endlog();
    return true;
}

bool ASCSmoothPathGenerator::startHook() {
    log(Info) << "[ASCSG] started!" << endlog();
    return true;
}

void ASCSmoothPathGenerator::updateHook() {
}

void ASCSmoothPathGenerator::stopHook() {
    log(Info) << "[ASCSG] stopped!" << endlog();
}

void ASCSmoothPathGenerator::cleanupHook() {
    log(Info) << "[ASCSG] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCSmoothPathGenerator)

}
}
