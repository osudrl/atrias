/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the asc_biped_boom_init subcontroller.
 */

#include <asc_biped_boom_init/controller_component.h>

namespace atrias {
namespace controller {

ASCBipedBoomInit::ASCBipedBoomInit(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log")
{
    this->provides("bipedBoomInit")
        ->addOperation("done", &ASCBipedBoomInit::done, this)
        .doc("Returns true if at the requested point");
    this->provides("bipedBoomInit")
        ->addOperation("run", &ASCBipedBoomInit::run, this);

    // Add properties
    this->addProperty("pd0Name", pd0Name);
    // Logging
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

    log(Info) << "[ASCBipedBoomInit] asc_biped_boom_init controller constructed!" << endlog();
}

bool ASCBipedBoomInit::done(void)
{
    // Are we done?
    return false;
}

atrias_msgs::controller_output ASCBipedBoomInit::run(atrias_msgs::robot_state rs, RobotPos pos)
{
    // Determine the robot config
    robotConfig = (uint8_t)rs.robotConfiguration;
    co.lLeg.motorCurrentA = 0.0;
    co.lLeg.motorCurrentB = 0.0;
    co.lLeg.motorCurrentHip = 0.0;
    co.rLeg.motorCurrentA = 0.0;
    co.rLeg.motorCurrentB = 0.0;
    co.rLeg.motorCurrentHip = 0.0;
    co.command = medulla_state_idle;

    // If nothing is going on
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::DISABLE)
    {
        return co;
    }
    else if (robotConfig == (uint8_t)rtOps::RobotConfiguration::UNKNOWN)
    {
        return co;
    }

    // We have a robot.  What is it?
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::LEFT_LEG_NOHIP)
    {
        // Set up the sin wave controllers
        // Set up the pd controllers

        P0.set(500.0);
        D0.set(20.0);

        targetPos = 1.0;
        currentPos = rs.lLeg.halfA.motorAngle;
        targetVel = 0.0;
        currentVel = rs.lLeg.halfA.motorVelocity;
        co.lLeg.motorCurrentA = pd0RunController(targetPos, currentPos, targetVel, currentVel);

        targetPos = 2.0;
        currentPos = rs.lLeg.halfB.motorAngle;
        targetVel = 0.0;
        currentVel = rs.lLeg.halfB.motorVelocity;
        co.lLeg.motorCurrentB = pd0RunController(targetPos, currentPos, targetVel, currentVel);
    }
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::LEFT_LEG_HIP)
    {
        // Nothing yet
    }
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::BIPED_NOHIP)
    {
        // Nothing yet
    }
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::BIPED_FULL)
    {
        // Nothing yet
    }
    co.command = medulla_state_run;
    return co;

}

bool ASCBipedBoomInit::configureHook() {
    // Connect to the subcontrollers
    pd0 = this->getPeer(pd0Name);
    if (pd0)
        pd0RunController = pd0->provides("pd")->getOperation("runController");

    // Get references to subcontroller component properties
    D0 = pd0->properties()->getProperty("D");
    P0 = pd0->properties()->getProperty("P");

    log(Info) << "[ASCBipedBoomInit] configured!" << endlog();
    return true;
}

bool ASCBipedBoomInit::startHook() {
    log(Info) << "[ASCBipedBoomInit] started!" << endlog();
    return true;
}

void ASCBipedBoomInit::updateHook() {
}

void ASCBipedBoomInit::stopHook() {
    log(Info) << "[ASCBipedBoomInit] stopped!" << endlog();
}

void ASCBipedBoomInit::cleanupHook() {
    log(Info) << "[ASCBipedBoomInit] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCBipedBoomInit)

}
}
