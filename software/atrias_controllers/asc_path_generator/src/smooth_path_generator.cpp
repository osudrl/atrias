/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the smooth path generator subcontroller.
 */

#include <asc_path_generator/smooth_path_generator.h>

namespace atrias {
namespace controller {

ASCSmoothPathGenerator::ASCSmoothPathGenerator(std::string name):
    RTT::TaskContext(name),
    timeElapsed(0.0),
    isFinished(true),
    logPort(name + "_log")
{
    this->provides("smoothPath")
        ->addOperation("runController", &ASCSmoothPathGenerator::runController, this, ClientThread)
        .doc("Run the controller.");
    this->provides("smoothPath")
        ->addOperation("init", &ASCSmoothPathGenerator::init, this, ClientThread);
    this->provides("smoothPath")
        ->addOperation("setTgt", &ASCSmoothPathGenerator::setTgt, this, ClientThread);
    this->addProperty("isFinished", isFinished)
        .doc("Has this finished?");

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

    log(Info) << name << " Constructed!" << endlog();
}

// Get start and end positions as well as the duration of time the motion
// should take.
void ASCSmoothPathGenerator::init(double startVal, double endVal, double durationVal) {
    // Setup the controller
    timeElapsed = 0.0;
    isFinished  = false;
    start       = startVal;
    end         = endVal;
    duration    = durationVal;
}

// Set a new endpoint -- can be done while running, if change in endpoint is continuous.
void ASCSmoothPathGenerator::setTgt(double endVal) {
	end = endVal;
}

// Output.
MotorState ASCSmoothPathGenerator::runController() {
    if (!isFinished) {
        output.ang = start + (end - start) * (0.5 * (sin((M_PI/duration) * timeElapsed - 0.5*M_PI)) + 0.5);
        output.vel = (end - start) * (0.5 * (cos((M_PI/duration) * timeElapsed - 0.5*M_PI)) * M_PI/duration);
        timeElapsed += 0.001;

        // If the elapsed time is greater than the requested duration, we are
        // done.
        if (timeElapsed >= duration) {
            // This may be unnecessary, but make sure the output is at the end value.
            output.ang = end;
            output.vel = 0.0;

            // And we are done.
            isFinished = true;
        }
    }

    // Stuff the msg and push to ROS for logging
    //logData.ang = output.ang;
    //logData.vel = output.vel;
    //logPort.write(logData);

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
