// Title: ASCHipBoomKinematics
// Description: Functions related to hip kinematics with robot on boom.
// Author: Mikhail Jones

// Usage: To use this subcontroller, add the following pieces of code to the the designated files in your controller folder.

// Manifest.xml
// <depend package="asc_hip_boom_kinematics"/>

// Start.ops
// # ASCHipBoomKinematics
// import("asc_hip_boom_kinematics")
// var string ascHipBoomKinematics0Name = atrias_cm.getUniqueName("controller", "ascHipBoomKinematics")
// loadComponent(ascHipBoomKinematics0Name, "ASCHipBoomKinematics")
// addPeer("controller", ascHipBoomKinematics0Name)
// connectPeers("Deployer", ascHipBoomKinematics0Name)
// controller.ascHipBoomKinematics0Name = ascHipBoomKinematics0Name

// Stop.ops
// # ASCHipBoomKinematics
// unloadComponent(controller.ascHipBoomKinematics0Name)

// component_controller.cpp
// In the first constructor function (has the same name as your controller)
// // ASCHipBoomKinematics
// this->addProperty("ascHipBoomKinematics0Name", ascHipBoomKinematics0Name);
// In the configure hook function
// // ASCHipInverseKinematics Service
// ascHipBoomKinematics0 = this->getPeer(ascHipBoomKinematics0Name);
// if (ascHipBoomKinematics0) {
// 	inverseKinematics0 = ascHipBoomKinematics0->provides("ascHipBoomKinematics")->getOperation("inverseKinematics");
// }

// component_controller.h
// // ASCHipBoomKinematics
// std::string ascHipBoomKinematics0Name;
// TaskContext *ascHipBoomKinematics0;
// OperationCaller<LeftRight(LeftRight, atrias_msgs::robot_state_leg, atrias_msgs::robot_state_leg, atrias_msgs::robot_state_location)> inverseKinematics0;

// To use do something like this.
// // Define toePosition struct.
// toePosition.left = 2.15;
// toePosition.right = 2.45;
//
// // Compute and set hipAngle.
// hipAngle = inverseKinematics0(toePosition, rs.lLeg, rs.rLeg, rs.position);


#include <asc_hip_boom_kinematics/controller_component.h>

namespace atrias {
namespace controller {


// ASCHipBoomKinematics ========================================================
ASCHipBoomKinematics::ASCHipBoomKinematics(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log") {
    
    // Add operations
	this->provides("ascHipBoomKinematics")
		->addOperation("inverseKinematics",&ASCHipBoomKinematics::inverseKinematics, this, ClientThread)
		.doc("Given a boom angle, leg angles, and desired toe positions, returns required hip angles");
       
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
    log(Info) << "[ASCHipBoomKinematics] Constructed!" << endlog();
    
}


// ASCHipBoomKinematics::inverseKinematics =====================================
LeftRight ASCHipBoomKinematics::inverseKinematics(LeftRight toePosition, atrias_msgs::robot_state_leg lLeg, atrias_msgs::robot_state_leg rLeg, atrias_msgs::robot_state_location position) {

    // Define imaginary number i.
    i = complex<double>(0.0, 1.0);

	// Define robot parameters.
	l1 = 0.50; // Leg thigh segment length.
	l2 = 0.50; // Leg shin segment length.
    lBoom = 2.04; // Distance from boom pivot Z axis to robot body XZ center plane along boom Y axis.
    lBody = 0.35; // Distance from boom Y axis intersection with robot body XZ center plane to hip pivot X axis.
    lHip = 0.18; // Distance from hip pivot X axis to XZ center plane of leg assembly.
    qBodyOffset = M_PI/2.0 - 0.126; // Angle between boom Y axis and robot body XZ center plane.

	// Get leg lengths and angles.
    lLeftLeg = (l1 + l2)*cos((lLeg.halfB.legAngle - lLeg.halfA.legAngle)/2.0);
    lRightLeg = (l1 + l2)*cos((rLeg.halfB.legAngle - rLeg.halfA.legAngle)/2.0);
    qLeftLeg = (lLeg.halfA.legAngle + lLeg.halfB.legAngle)/2.0;
    qRightLeg = (rLeg.halfA.legAngle + rLeg.halfB.legAngle)/2.0;

	// Compute inverse kinematics
	complexHipAngleLeft = - position.boomAngle - qBodyOffset - log((- sqrt(pow(lLeftLeg, 2) - 2.0*pow(lLeftLeg, 2)*exp(qLeftLeg*2.0*i) + pow(lLeftLeg, 2)*exp(qLeftLeg*4.0*i) + 4.0*pow(toePosition.left, 2)*exp(qLeftLeg*2.0*i) - 4.0*pow(lHip, 2)*exp(qLeftLeg*2.0*i) + 4.0*pow(lBoom, 2)*exp(qLeftLeg*2.0*i)*pow(cos(position.boomAngle), 2) - 4.0*pow(lLeftLeg, 2)*exp(qLeftLeg*2.0*i)*pow(cos(qLeftLeg), 2) + 4.0*pow(lBody, 2)*exp(qLeftLeg*2.0*i)*pow(cos(position.boomAngle), 2)*pow(cos(qBodyOffset), 2) + 4.0*pow(lBody, 2)*exp(qLeftLeg*2.0*i)*pow(sin(position.boomAngle), 2)*pow(sin(qBodyOffset), 2) + 8.0*lBoom*exp(qLeftLeg*2.0*i)*cos(position.boomAngle)*sqrt(toePosition.left + lLeftLeg*cos(qLeftLeg))*sqrt(toePosition.left - lLeftLeg*cos(qLeftLeg)) + 8.0*lBoom*lBody*exp(qLeftLeg*2.0*i)*pow(cos(position.boomAngle), 2)*cos(qBodyOffset) - 8.0*lBoom*lBody*exp(qLeftLeg*2.0*i)*cos(position.boomAngle)*sin(position.boomAngle)*sin(qBodyOffset) + 8.0*lBody*exp(qLeftLeg*2.0*i)*cos(position.boomAngle)*cos(qBodyOffset)*sqrt(toePosition.left + lLeftLeg*cos(qLeftLeg))*sqrt(toePosition.left - lLeftLeg*cos(qLeftLeg)) - 8.0*lBody*exp(qLeftLeg*2.0*i)*sin(position.boomAngle)*sin(qBodyOffset)*sqrt(toePosition.left + lLeftLeg*cos(qLeftLeg))*sqrt(toePosition.left - lLeftLeg*cos(qLeftLeg)) - 8.0*pow(lBody, 2)*exp(qLeftLeg*2.0*i)*cos(position.boomAngle)*cos(qBodyOffset)*sin(position.boomAngle)*sin(qBodyOffset)) + 2.0*lBoom*cos(position.boomAngle)*(cos(qLeftLeg) + sin(qLeftLeg)*i) + 2.0*exp(qLeftLeg*i)*sqrt(toePosition.left + lLeftLeg*cos(qLeftLeg))*sqrt(toePosition.left - lLeftLeg*cos(qLeftLeg)) + 2.0*lBody*cos(position.boomAngle + qBodyOffset)*(cos(qLeftLeg) + sin(qLeftLeg)*i))/(lLeftLeg + 2.0*lHip*exp(qLeftLeg*i) - lLeftLeg*exp(qLeftLeg*2.0*i)))*i;

    complexHipAngleRight = - position.boomAngle - qBodyOffset - log(-(- sqrt(2.0*pow(lBody, 2)*exp(qRightLeg*2.0*i) - 4.0*pow(lHip, 2)*exp(qRightLeg*2.0*i) - 2.0*pow(lRightLeg, 2)*exp(qRightLeg*2.0*i) + pow(lRightLeg, 2)*exp(qRightLeg*4.0*i) + 4.0*pow(toePosition.right, 2)*exp(qRightLeg*2.0*i) + pow(lRightLeg, 2) + 4.0*pow(lBoom, 2)*exp(qRightLeg*2.0*i)*pow(cos(position.boomAngle), 2) - 4.0*pow(lRightLeg, 2)*exp(qRightLeg*2.0*i)*pow(cos(qRightLeg), 2) + 2.0*pow(lBody, 2)*cos(2.0*position.boomAngle + 2.0*qBodyOffset)*exp(qRightLeg*2.0*i) + 4.0*lBoom*lBody*exp(qRightLeg*2.0*i)*cos(qBodyOffset) + 4.0*lBoom*lBody*cos(2.0*position.boomAngle + qBodyOffset)*exp(qRightLeg*2.0*i) + 8.0*lBody*exp(qRightLeg*2.0*i)*cos(position.boomAngle + qBodyOffset)*sqrt(toePosition.right + lRightLeg*cos(qRightLeg))*sqrt(toePosition.right - lRightLeg*cos(qRightLeg)) + 8.0*lBoom*exp(qRightLeg*2.0*i)*cos(position.boomAngle)*sqrt(toePosition.right + lRightLeg*cos(qRightLeg))*sqrt(toePosition.right - lRightLeg*cos(qRightLeg))) + 2.0*exp(qRightLeg*i)*sqrt(toePosition.right + lRightLeg*cos(qRightLeg))*sqrt(toePosition.right - lRightLeg*cos(qRightLeg)) + 2.0*lBody*exp(qRightLeg*i)*cos(position.boomAngle + qBodyOffset) + 2.0*lBoom*exp(qRightLeg*i)*cos(position.boomAngle))/(- lRightLeg + 2.0*lHip*exp(qRightLeg*i) + lRightLeg*exp(qRightLeg*2.0*i)))*i;

	// TODO - Add velocity terms

	// We only care about the real part, the imaginary part should be zero.
	hipAngle.left = fmod(real(complexHipAngleLeft) + 4.0*M_PI, 2.0*M_PI);
	hipAngle.right = fmod(real(complexHipAngleRight) + 4.0*M_PI, 2.0*M_PI);

	// TODO - Clamp hip angles to physical limits.
	
	// Stuff the msg and push to ROS for logging
	logData.header = getROSHeader();
    logData.leftHipAngle = hipAngle.left;
    logData.rightHipAngle = hipAngle.right;
    logPort.write(logData);
	
	return hipAngle;

} // inverseKinematics


// configureHook ===============================================================
bool ASCHipBoomKinematics::configureHook() {
	// Log data stuff
    RTT::TaskContext* rtOpsPeer = this->getPeer("Deployer")->getPeer("atrias_rt");
    if (rtOpsPeer) {
        getROSHeader = rtOpsPeer->provides("timestamps")->getOperation("getROSHeader");
    } else {
        log(Warning) << "[ASCHipBoomKinematics] Can't connect to the Deployer" << endlog();
    }

    log(Info) << "[ASCHipBoomKinematics] configured!" << endlog();
    return true;
} // configureHook


// startHook ===================================================================
bool ASCHipBoomKinematics::startHook() {
    log(Info) << "[ASCHipBoomKinematics] started!" << endlog();
    return true;
} // startHook


// updateHook ==================================================================
void ASCHipBoomKinematics::updateHook() {
} // updateHook


// stopHook ====================================================================
void ASCHipBoomKinematics::stopHook() {
    log(Info) << "[ASCHipBoomKinematics] stopped!" << endlog();
} // stopHook


// cleanupHook =================================================================
void ASCHipBoomKinematics::cleanupHook() {
    log(Info) << "[ASCHipBoomKinematics] cleaned up!" << endlog();
} // cleanupHook


ORO_CREATE_COMPONENT(ASCHipBoomKinematics)

} // namespace controller
} // namespace atrias
