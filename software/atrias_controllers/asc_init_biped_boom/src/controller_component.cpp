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
        ->addOperation("init", &ASCInitBipedBoom::init, this);
    this->provides("bipedBoomInit")
        ->addOperation("run", &ASCInitBipedBoom::run, this);
    this->provides("bipedBoomInit")
        ->addOperation("done", &ASCInitBipedBoom::done, this)
        .doc("Returns true if at the requested point");

    // Add properties
    this->addProperty("pd0Name", pd0Name);
    this->addProperty("smoothPath0Name", smoothPath0Name);
    this->addProperty("smoothPath1Name", smoothPath1Name);
    this->addProperty("smoothPath2Name", smoothPath2Name);
    this->addProperty("smoothPath3Name", smoothPath3Name);
    this->addProperty("smoothPath4Name", smoothPath4Name);
    this->addProperty("smoothPath5Name", smoothPath5Name);

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

bool ASCInitBipedBoom::init(atrias_msgs::robot_state rs, RobotPos pos)
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
        smoothPath0Init(rs.lLeg.halfA.motorAngle, pos.lLeg.A);
        smoothPath1Init(rs.lLeg.halfB.motorAngle, pos.lLeg.B);
    }
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::LEFT_LEG_HIP)
    {
        smoothPath0Init(rs.lLeg.halfA.motorAngle, pos.lLeg.A);
        smoothPath1Init(rs.lLeg.halfB.motorAngle, pos.lLeg.B);
        smoothPath2Init(rs.lLeg.hip.legBodyAngle, pos.lLeg.hip);
    }
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::BIPED_NOHIP)
    {
        smoothPath0Init(rs.lLeg.halfA.motorAngle, pos.lLeg.A);
        smoothPath1Init(rs.lLeg.halfB.motorAngle, pos.lLeg.B);
        smoothPath3Init(rs.rLeg.halfA.motorAngle, pos.rLeg.A);
        smoothPath4Init(rs.rLeg.halfB.motorAngle, pos.rLeg.B);
    }
    if (robotConfig == (uint8_t)rtOps::RobotConfiguration::BIPED_FULL)
    {
        smoothPath0Init(rs.lLeg.halfA.motorAngle, pos.lLeg.A);
        smoothPath1Init(rs.lLeg.halfB.motorAngle, pos.lLeg.B);
        smoothPath2Init(rs.lLeg.hip.legBodyAngle, pos.lLeg.hip);
        smoothPath3Init(rs.rLeg.halfA.motorAngle, pos.rLeg.A);
        smoothPath4Init(rs.rLeg.halfB.motorAngle, pos.rLeg.B);
        smoothPath5Init(rs.rLeg.hip.legBodyAngle, pos.rLeg.hip);
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
        frequency = 1; // 1 Hz
        amplitude = 1; // TODO: remove this.  It is not necessary
        lLegAState = smoothPath0Run(frequency, amplitude);
        lLegBState = smoothPath1Run(frequency, amplitude);

        // Run the pd controllers
        P0.set(600.0);
        D0.set(20.0);

        targetPos = lLegAState.ang;
        currentPos = rs.lLeg.halfA.motorAngle;
        targetVel = lLegAState.vel;
        currentVel = rs.lLeg.halfA.motorVelocity;
        co.lLeg.motorCurrentA = pd0RunController(targetPos, currentPos, targetVel, currentVel);

        targetPos = lLegBState.ang;
        currentPos = rs.lLeg.halfB.motorAngle;
        targetVel = lLegBState.vel;
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

    smoothPath0 = this->getPeer(smoothPath0Name);
    if (smoothPath0)
    {
        smoothPath0Init = smoothPath0->provides("smoothPath")->getOperation("init");
        smoothPath0Run = smoothPath0->provides("smoothPath")->getOperation("run");
    }

    smoothPath1 = this->getPeer(smoothPath1Name);
    if (smoothPath1)
    {
        smoothPath1Init = smoothPath1->provides("smoothPath")->getOperation("init");
        smoothPath1Run = smoothPath1->provides("smoothPath")->getOperation("run");
    }

    smoothPath2 = this->getPeer(smoothPath2Name);
    if (smoothPath2)
    {
        smoothPath2Init = smoothPath2->provides("smoothPath")->getOperation("init");
        smoothPath2Run = smoothPath2->provides("smoothPath")->getOperation("run");
    }

    smoothPath3 = this->getPeer(smoothPath3Name);
    if (smoothPath3)
    {
        smoothPath3Init = smoothPath3->provides("smoothPath")->getOperation("init");
        smoothPath3Run = smoothPath3->provides("smoothPath")->getOperation("run");
    }

    smoothPath4 = this->getPeer(smoothPath4Name);
    if (smoothPath4)
    {
        smoothPath4Init = smoothPath4->provides("smoothPath")->getOperation("init");
        smoothPath4Run = smoothPath4->provides("smoothPath")->getOperation("run");
    }

    smoothPath5 = this->getPeer(smoothPath5Name);
    if (smoothPath5)
    {
        smoothPath5Init = smoothPath5->provides("smoothPath")->getOperation("init");
        smoothPath5Run = smoothPath5->provides("smoothPath")->getOperation("run");
    }

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
