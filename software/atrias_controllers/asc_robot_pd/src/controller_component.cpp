/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the robot position controller.
 */

#include <asc_robot_pd/controller_component.h>

namespace atrias {
namespace controller {

ASCRobotPd::ASCRobotPd(std::string name):
    RTT::TaskContext(name)
{
    this->provides("robot_pd")
        ->addOperation("runController", &ASCRobotPd::runController, this, ClientThread)
        .doc("Get a desired robot state and return a controller output.");

    // Add properties.
    this->addProperty("pd0Name", pd0Name);
    this->addProperty("pd1Name", pd1Name);
    this->addProperty("pd2Name", pd2Name);
    this->addProperty("pd3Name", pd3Name);
    this->addProperty("pd4Name", pd4Name);
    this->addProperty("pd5Name", pd5Name);
    this->addProperty("legP", legP);
    this->addProperty("legD", legD);
    this->addProperty("hipP", hipP);
    this->addProperty("hipD", hipD);

    // Set Defaults
    legP       = 0.0;
    legD       = 0.0;
    hipP       = 0.0;
    hipD       = 0.0;
    targetPos  = 0.0;
    currentPos = 0.0;
    targetVel  = 0.0;
    currentVel = 0.0;

    log(Info) << "[ASCRobotPd] controller constructed!" << endlog();
}

// Put control code here.
atrias_msgs::controller_output ASCRobotPd::runController(atrias_msgs::robot_state rs, DesiredRobotState ds) {
    // Set the PD gains
    // lLeg
    // MotorA
    P0.set(legP);
    D0.set(legD);
    // MotorB
    P1.set(legP);
    D1.set(legD);
    // Hip
    P2.set(hipP);
    D2.set(hipD);
    // rLeg
    // MotorA
    P3.set(legP);
    D3.set(legD);
    // MotorB
    P4.set(legP);
    D4.set(legD);
    // Hip
    P5.set(hipP);
    D5.set(hipD);

    // Structs for the transforms
    lLegAng.ang = ds.left.leg.ang;
    lLegAng.vel = ds.left.leg.angVel;
    lLegLen.ang = ds.left.leg.len;
    lLegLen.vel = ds.left.leg.lenVel;
    rLegAng.ang = ds.right.leg.ang;
    rLegAng.vel = ds.right.leg.angVel;
    rLegLen.ang = ds.right.leg.len;
    rLegLen.vel = ds.right.leg.lenVel;
    // Transform from leg to motor positions
    lMotorAng = legToMotorPos(ds.left.leg.ang,  ds.left.leg.len);
    rMotorAng = legToMotorPos(ds.right.leg.ang, ds.right.leg.len);
    // Transform from leg to motor velocities
    lMotorVel = legToMotorVel(lLegAng, lLegLen);
    rMotorVel = legToMotorVel(rLegAng, rLegLen);

    // lLeg
    // Calculate motorA current
    targetPos  = lMotorAng.A;
    currentPos = rs.lLeg.halfA.motorAngle;
    targetVel  = lMotorVel.A;
    currentVel = rs.lLeg.halfA.motorVelocity;
    co.lLeg.motorCurrentA = pd0Controller(targetPos, currentPos, targetVel, currentVel);

    // Calculate motorB current
    targetPos  = lMotorAng.B;
    currentPos = rs.lLeg.halfB.motorAngle;
    targetVel  = lMotorVel.B;
    currentVel = rs.lLeg.halfB.motorVelocity;
    co.lLeg.motorCurrentB = pd1Controller(targetPos, currentPos, targetVel, currentVel);

    // Calculate hip current
    targetPos  = ds.left.hip.ang;
    currentPos = rs.lLeg.hip.legBodyAngle;
    targetVel  = ds.left.hip.vel;
    currentVel = rs.lLeg.hip.legBodyVelocity;
    co.lLeg.motorCurrentHip = pd2Controller(targetPos, currentPos, targetVel, currentVel);

    // rLeg
    // Calculate motorA current
    targetPos  = rMotorAng.A;
    currentPos = rs.rLeg.halfA.motorAngle;
    targetVel  = rMotorVel.A;
    currentVel = rs.rLeg.halfA.motorVelocity;
    co.rLeg.motorCurrentA = pd3Controller(targetPos, currentPos, targetVel, currentVel);

    // Calculate motorB current
    targetPos  = rMotorAng.B;
    currentPos = rs.rLeg.halfB.motorAngle;
    targetVel  = rMotorVel.A;
    currentVel = rs.rLeg.halfB.motorVelocity;
    co.rLeg.motorCurrentB = pd4Controller(targetPos, currentPos, targetVel, currentVel);

    // Calculate hip current
    targetPos  = ds.right.hip.ang;
    currentPos = rs.rLeg.hip.legBodyAngle;
    targetVel  = ds.right.hip.vel;
    currentVel = rs.rLeg.hip.legBodyVelocity;
    co.rLeg.motorCurrentHip = pd5Controller(targetPos, currentPos, targetVel, currentVel);

    co.command = medulla_state_run;

    return co;
}

// Don't put control code below here!
bool ASCRobotPd::configureHook() {
    // Connect to the subcontrollers
    pd0 = this->getPeer(pd0Name);
    if (pd0)
        pd0Controller = pd0->provides("pd")->getOperation("runController");

    pd1 = this->getPeer(pd1Name);
    if (pd1)
        pd1Controller = pd1->provides("pd")->getOperation("runController");

    pd2 = this->getPeer(pd2Name);
    if (pd2)
        pd2Controller = pd2->provides("pd")->getOperation("runController");

    pd3 = this->getPeer(pd3Name);
    if (pd3)
        pd3Controller = pd3->provides("pd")->getOperation("runController");

    pd4 = this->getPeer(pd4Name);
    if (pd4)
        pd4Controller = pd4->provides("pd")->getOperation("runController");

    pd5 = this->getPeer(pd5Name);
    if (pd5)
        pd5Controller = pd5->provides("pd")->getOperation("runController");

    legToMotorPos = this->provides("legToMotorTransforms")->getOperation("posTransform");
    legToMotorVel = this->provides("legToMotorTransforms")->getOperation("velTransform");

    // Get references to the attributes
    P0 = pd0->properties()->getProperty("P");
    D0 = pd0->properties()->getProperty("D");
    P1 = pd1->properties()->getProperty("P");
    D1 = pd1->properties()->getProperty("D");
    P2 = pd2->properties()->getProperty("P");
    D2 = pd2->properties()->getProperty("D");
    P3 = pd3->properties()->getProperty("P");
    D3 = pd3->properties()->getProperty("D");
    P4 = pd4->properties()->getProperty("P");
    D4 = pd4->properties()->getProperty("D");
    P5 = pd5->properties()->getProperty("P");
    D5 = pd5->properties()->getProperty("D");

    log(Info) << "[ASCRobotPd] configured!" << endlog();
    return true;
}

bool ASCRobotPd::startHook() {
    log(Info) << "[ASCRobotPd] started!" << endlog();
    return true;
}

void ASCRobotPd::updateHook() {
    // Do nothing
}

void ASCRobotPd::stopHook() {
    log(Info) << "[ASCRobotPd] stopped!" << endlog();
}

void ASCRobotPd::cleanupHook() {
    log(Info) << "[ASCRobotPd] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(ASCRobotPd)

}
}

