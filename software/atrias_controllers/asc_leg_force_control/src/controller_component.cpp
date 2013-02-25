// Title: ASCLegForceControl
// Description: Defines leg force to motor torque relationships.
// Author: Mikhail Jones

// Usage: To use this subcontroller, add the following pieces of code to the the designated files in your controller folder.

// Manifest.xml
// <depend package="asc_leg_force_control"/>

// Start.ops
// # ASCLegForceControl
// import("asc_leg_force_control")
// var string ascLegForceControl0Name = atrias_cm.getUniqueName("controller", "ascLegForceControl")
// loadComponent(ascLegForceControl0Name, "ASCLegForceControl")
// addPeer("controller", ascLegForceControl0Name)
// connectPeers("Deployer", ascLegForceControl0Name)
// controller.ascLegForceControl0Name = ascLegForceControl0Name

// Stop.ops
// # ASCLegForceControl
// unloadComponent(controller.ascLegForceControl0Name)

// component_controller.cpp
// In the first constructor function (has the same name as your controller)
// // ASCLegForceControl
// this->addProperty("ascLegForceControl0Name", ascLegForceControl0Name);
// In the configure hook function
// // ASCLegForceControl Service
// ascLegForceControl0 = this->getPeer(ascLegForceControl0Name);
// if (ascLegForceControl0) {
// 	legForceToMotorCurrent0 = ascLegForceControl0->provides("ascLegForceControl")->getOperation("legForceToMotorCurrent");
// }

// component_controller.h
// // ASCLegForceControl
// std::string ascLegForceControl0Name;
// TaskContext *ascLegForceControl0;
// OperationCaller<AB(LegForce, Gain, atrias_msgs::robot_state_leg, atrias_msgs::robot_state_location)> legForceToMotorCurrent0;

// To use do something like this.
//
// // Define gain struct
// gain.kp = 1000.0;
// gain.kd = 8.0;
// gain.ks = 4118.0;
// gain.kg = 50.0;    
// gain.kt = 0.0987;
//
// // Define legForce struct.
// legForce.fx = 0.0;
// legForce.fz = 0.0;
// legForce.dfx = 0.0;
// legForce.dfz = 0.0;
//
// // Compute and set required motorCurrent
// motorCurrent = legForceToMotorCurrent(legForce, gain, rs.lLeg, rs.position);
// co.lLeg.motorCurrentA = motorCurrent.A;
// co.lLeg.motorCurrentB = motorCurrent.B;


#include <asc_leg_force_control/controller_component.h>

namespace atrias {
namespace controller {


// ASCLegForceControl ==========================================================
ASCLegForceControl::ASCLegForceControl(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log") {
    
    // Add operations
    this->provides("ascLegForceControl")->addOperation("legForceToMotorCurrent", &ASCLegForceControl::legForceToMotorCurrent, this).doc("Given a desired X-Z component force, returns the required motor current.");

    // LOGGING
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
    log(Info) << "[ASCLegForceControl] Constructed!" << endlog();
    
} // ASCLegForceControl


// legForceToMotorCurrent ======================================================
AB ASCLegForceControl::legForceToMotorCurrent(LegForce legForce, Gain gain, atrias_msgs::robot_state_leg leg, atrias_msgs::robot_state_location position) {

	// Declare robot parameters.
	l1 = 0.50;
	l2 = 0.50;

	// Compute required joint torque using Jacobian.
	tauSpringA = -legForce.fx*l2*cos(leg.halfA.legAngle + position.bodyPitch) + legForce.fz*l2*sin(leg.halfA.legAngle + position.bodyPitch);
	tauSpringB = -legForce.fx*l1*cos(leg.halfB.legAngle + position.bodyPitch) + legForce.fz*l1*sin(leg.halfB.legAngle + position.bodyPitch);

	// Compute required differential joint torque.
	dtauSpringA = legForce.fx*l2*sin(leg.halfA.legAngle + position.bodyPitch)*(leg.halfA.legVelocity + position.bodyPitchVelocity) + legForce.dfz*l2*sin(leg.halfA.legAngle + position.bodyPitch) - legForce.dfx*l2*cos(leg.halfA.legAngle + position.bodyPitch) + legForce.fz*l2*cos(leg.halfA.legAngle + position.bodyPitch)*(leg.halfA.legVelocity + position.bodyPitchVelocity);
	dtauSpringB = legForce.fx*l1*sin(leg.halfB.legAngle + position.bodyPitch)*(leg.halfB.legVelocity + position.bodyPitchVelocity) + legForce.dfz*l1*sin(leg.halfB.legAngle + position.bodyPitch) - legForce.dfx*l1*cos(leg.halfB.legAngle + position.bodyPitch) + legForce.fz*l1*cos(leg.halfB.legAngle + position.bodyPitch)*(leg.halfB.legVelocity + position.bodyPitchVelocity);

	// Compute required motor current using PD controller with feed forward term.
	motorCurrent.A = (gain.ks*(leg.halfA.motorAngle - leg.halfA.legAngle)/gain.kg + gain.kp*(tauSpringA/gain.ks - (leg.halfA.motorAngle - leg.halfA.legAngle)) + gain.kd*(dtauSpringA/gain.ks - (leg.halfA.motorVelocity - leg.halfA.legVelocity)))/gain.kt;
	motorCurrent.B = (gain.ks*(leg.halfB.motorAngle - leg.halfB.legAngle)/gain.kg + gain.kp*(tauSpringB/gain.ks - (leg.halfB.motorAngle - leg.halfB.legAngle)) + gain.kd*(dtauSpringB/gain.ks - (leg.halfB.motorVelocity - leg.halfB.legVelocity)))/gain.kt;

    // Stuff the msg and push to ROS for logging
	logData.header = getROSHeader();
	logData.fx = legForce.fx;
	logData.fz = legForce.fz;
	logData.tauSpringA = tauSpringA;
	logData.tauSpringB = tauSpringB;
	logData.dtauSpringA = dtauSpringA;
	logData.dtauSpringB = dtauSpringB;
    logPort.write(logData);

	return motorCurrent;

} // legForceToMotorCurrent


// configureHook ===============================================================
bool ASCLegForceControl::configureHook() {
	// Log data stuff
    RTT::TaskContext* rtOpsPeer = this->getPeer("Deployer")->getPeer("atrias_rt");
    if (rtOpsPeer) {
        getROSHeader = rtOpsPeer->provides("timestamps")->getOperation("getROSHeader");
    } else {
        log(Warning) << "[ASCLegForceControl] Can't connect to the Deployer" << endlog();
    }
    
    log(Info) << "[ASCLegForceControl] configured!" << endlog();
    return true;
} // configureHook


// startHook ===================================================================
bool ASCLegForceControl::startHook() {
    log(Info) << "[ASCLegForceControl] started!" << endlog();
    return true;
} // startHook


// updateHook ==================================================================
void ASCLegForceControl::updateHook() {
} // updateHook


// stopHook ====================================================================
void ASCLegForceControl::stopHook() {
    log(Info) << "[ASCLegForceControl] stopped!" << endlog();
} // stopHook


// cleanupHook =================================================================
void ASCLegForceControl::cleanupHook() {
    log(Info) << "[ASCLegForceControl] cleaned up!" << endlog();
} //cleanupHook


ORO_CREATE_COMPONENT(ASCLegForceControl)

} // namespace controller
} // namespace atrias
