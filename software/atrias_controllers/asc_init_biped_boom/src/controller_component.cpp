/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the asc_init_biped_boom subcontroller.
 */

#include <asc_init_biped_boom/controller_component.h>

namespace atrias {
namespace controller {

ASCInitBipedBoom::ASCInitBipedBoom(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log")
{
    this->provides("bipedBoomInit")
        ->addOperation("done", &ASCInitBipedBoom::done, this)
        .doc("Returns true if at the requested point");
    this->provides("bipedBoomInit")
        ->addOperation("run", &ASCInitBipedBoom::run, this);

    // Add properties
    this->addProperty("pd0Name", pd0Name);
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

    log(Info) << "[ASCInitBipedBoom] asc_init_biped_boom controller constructed!" << endlog();
}

bool ASCInitBipedBoom::done(void)
{
    // Are we done?
    return false;
}

bool ASCInitBipedBoom:init(atrias_msgs::robot_state rs, RobotPos pos)
{
    robotConfig = (uint8_t)rs.robotConfiguration;

    // If nothing is going on
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::DISABLE)
        return false;
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::UNKNOWN)
        return false;

    // We have a robot.  What is it?
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::LEFT_LEG_NOHIP)
    {
        pathGen0Init(rs.lLeg.halfA.motorAngle, pos.lLeg.A);
        pathGen1Init(rs.lLeg.halfB.motorAngle, pos.lLeg.B);
    }
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::LEFT_LEG_HIP)
    {
        pathGen0Init(rs.lLeg.halfA.motorAngle, pos.lLeg.A);
        pathGen1Init(rs.lLeg.halfB.motorAngle, pos.lLeg.B);
        pathGen2Init(rs.lLeg.hip.legBodyAngle, pos.lLeg.hip);
    }
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::BIPED_NOHIP)
    {
        pathGen0Init(rs.lLeg.halfA.motorAngle, pos.lLeg.A);
        pathGen1Init(rs.lLeg.halfB.motorAngle, pos.lLeg.B);
        pathGen3Init(rs.rLeg.halfA.motorAngle, pos.rLeg.A);
        pathGen4Init(rs.rLeg.halfB.motorAngle, pos.rLeg.B);
    }
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::BIPED_FULL)
    {
        pathGen0Init(rs.lLeg.halfA.motorAngle, pos.lLeg.A);
        pathGen1Init(rs.lLeg.halfB.motorAngle, pos.lLeg.B);
        pathGen2Init(rs.lLeg.hip.legBodyAngle, pos.lLeg.hip);
        pathGen3Init(rs.rLeg.halfA.motorAngle, pos.rLeg.A);
        pathGen4Init(rs.rLeg.halfB.motorAngle, pos.rLeg.B);
        pathGen5Init(rs.rLeg.hip.legBodyAngle, pos.rLeg.hip);
    }
    return true;
}

atrias_msgs::controller_output ASCInitBipedBoom::run(atrias_msgs::robot_state rs)
{
    // Determine the robot config
    robotConfig             = (uint8_t)rs.robotConfiguration;
    co.lLeg.motorCurrentA   = 0.0;
    co.lLeg.motorCurrentB   = 0.0;
    co.lLeg.motorCurrentHip = 0.0;
    co.rLeg.motorCurrentA   = 0.0;
    co.rLeg.motorCurrentB   = 0.0;
    co.rLeg.motorCurrentHip = 0.0;

    // If nothing is going on
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::DISABLE)
        return co;
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::UNKNOWN)
        return co;

    // We have a robot.  What is it?
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::LEFT_LEG_NOHIP)
    {
        // Run the path generator
        lLegAPath = pathGen0Run();
        lLegBPath = pathGen1Run();

        // Run the pd controllers
        P0.set(600.0);
        D0.set(20.0);

        targetPos = lLegAPath.ang;
        currentPos = rs.lLeg.halfA.motorAngle;
        targetVel = lLegAPath.vel;
        currentVel = rs.lLeg.halfA.motorVelocity;
        co.lLeg.motorCurrentA = pd0RunController(targetPos, currentPos, targetVel, currentVel);

        targetPos = lLegBPath.ang;
        currentPos = rs.lLeg.halfB.motorAngle;
        targetVel = lLegBPath.vel;
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

bool ASCInitBipedBoom::configureHook() {
    // Connect to the subcontrollers
    pd0 = this->getPeer(pd0Name);
    if (pd0)
        pd0RunController = pd0->provides("pd")->getOperation("runController");

    // Get references to subcontroller component properties
    D0 = pd0->properties()->getProperty("D");
    P0 = pd0->properties()->getProperty("P");

    log(Info) << "[ASCInitBipedBoom] configured!" << endlog();
    return true;
}

bool ASCInitBipedBoom::startHook() {
    log(Info) << "[ASCInitBipedBoom] started!" << endlog();
    return true;
}

void ASCInitBipedBoom::updateHook() {
}

void ASCInitBipedBoom::stopHook() {
    log(Info) << "[ASCInitBipedBoom] stopped!" << endlog();
}

void ASCInitBipedBoom::cleanupHook() {
    log(Info) << "[ASCInitBipedBoom] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCInitBipedBoom)

}
}
