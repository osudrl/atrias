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
        ->addOperation("passRobotState", &ASCBipedBoomInit::passRobotState, this)
        .doc("Pass the current robot state for use in the other operations");
    this->provides("bipedBoomInit")
        ->addOperation("isInitialized", &ASCBipedBoomInit::isInitialized, this)
        .doc("Returns true if at the requested point");
    this->provides("bipedBoomInit")
        ->addOperation("leftLeg", &ASCBipedBoomInit::leftLeg, this);
//    this->provides("bipedBoomInit")
//      ->addOperation("rightLeg", &ASCBipedBoomInit::rightLeg, this);
//    this->provides("bipedBoomInit")
//      ->addOperation("leftHip", &ASCBipedBoomInit::leftHip, this);
//    this->provides("bipedBoomInit")
//      ->addOperation("rightHip", &ASCBipedBoomInit::rightHip, this);

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

void ASCBipedBoomInit::passRobotState(atrias_msgs::robot_state _rs)
{
    // Set the gains
    P0.set(500.0);
    D0.set(20.0);

    rs = _rs;
    // Stuff the msg and push to ROS for logging
    //logData.input = exampleInput;
    //logData.output = out;
    //logPort.write(logData);
    return;
}

int ASCBipedBoomInit::isInitialized(void)
{
    stateCheck=0;
    return stateCheck;
}

MotorCurrent ASCBipedBoomInit::leftLeg(double aTargetPos, double bTargetPos)
{
    targetPos = aTargetPos;
    currentPos = rs.lLeg.halfA.motorAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.halfA.motorVelocity;
    motorCurrent.A = pd0RunController(targetPos, currentPos, targetVel, currentVel);

    targetPos = bTargetPos;
    currentPos = rs.lLeg.halfB.motorAngle;
    targetVel = 0.0;
    currentVel = rs.lLeg.halfB.motorVelocity;
    motorCurrent.B = pd0RunController(targetPos, currentPos, targetVel, currentVel);

    // Current cap
    cap = 5.0;
    motorCurrent.A = ASCBipedBoomInit::capCurrent(motorCurrent.A);
    motorCurrent.B = ASCBipedBoomInit::capCurrent(motorCurrent.B);

    return motorCurrent;
}

double ASCBipedBoomInit::capCurrent(double current)
{
    // Cap the current
    if ( current > cap )
        current = cap;
    else if ( current < -cap )
        current = -cap;

    return current;
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
