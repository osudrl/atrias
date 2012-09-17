/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the sin path generator subcontroller.
 */

#include <asc_path_generator/sin_path_generator.h>

namespace atrias {
namespace controller {

ASCSinPathGenerator::ASCSinPathGenerator(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log")
{
    this->provides("sinGen")
        ->addOperation("runController", &ASCSinPathGenerator::runController, this, OwnThread)
        .doc("Run the controller.");
    this->provides("sinGen")
        ->addOperation("reset", &ASCSinPathGenerator::reset, this, OwnThread);
    this->provides("sinGen")
        ->addOperation("setPhase", &ASCSinPathGenerator::setPhase, this, OwnThread);

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
    phase = 0.0;

    // Debugging
    count = 0;

    log(Info) << "[ASCSG] Constructed!" << endlog();
}

void ASCSinPathGenerator::reset(void)
{
    // Reset the accumulator
    accumulator = 0.0;
}

void ASCSinPathGenerator::setPhase(double _phase)
{
    // Set the phase
    phase = _phase;
}

MotorState ASCSinPathGenerator::runController(double frequency, double amplitude)
{
    // Return the function value and its derivative
    sinOut.ang = amplitude*sin(2.0*M_PI*(accumulator + phase));
    sinOut.vel = amplitude*cos(2.0*M_PI*(accumulator + phase))*2.0*M_PI*frequency;

    if (count < 1)
    {
        printf("Sin calculation:\n");
        printf("Sin output = %f\n", sinOut.ang);
        printf("Amplitude  = %f\n", amplitude);
        count++;
    }

    // The sin input
    accumulator += 0.001*frequency;
    if (accumulator >= 1.0)
        accumulator -= 1.0;

    // Stuff the msg and push to ROS for logging
    logData.ang = sinOut.ang;
    logData.vel = sinOut.vel;
    logPort.write(logData);

    return sinOut;
}

bool ASCSinPathGenerator::configureHook() {
    log(Info) << "[ASCSG] configured!" << endlog();
    return true;
}

bool ASCSinPathGenerator::startHook() {
    log(Info) << "[ASCSG] started!" << endlog();
    return true;
}

void ASCSinPathGenerator::updateHook() {
}

void ASCSinPathGenerator::stopHook() {
    log(Info) << "[ASCSG] stopped!" << endlog();
}

void ASCSinPathGenerator::cleanupHook() {
    log(Info) << "[ASCSG] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCSinPathGenerator)

}
}
