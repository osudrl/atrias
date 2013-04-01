// Title: ASCCommonToolkit
// Description: Provides commonly used functions and tools
// Author: Mikhail Jones

// Usage: To use this subcontroller, add the following pieces of code to the the designated files in your controller folder.

// Manifest.xml
// <depend package="asc_common_toolkit"/>

// Start.ops
// # ASCCommonToolkit
// import("asc_common_toolkit")
// var string ascCommonToolkit0Name = atrias_cm.getUniqueName("controller", "ascCommonToolkit")
// loadComponent(ascCommonToolkit0Name, "ASCCommonToolkit")
// addPeer("controller", ascCommonToolkit0Name)
// connectPeers("Deployer", ascCommonToolkit0Name)
// controller.ascCommonToolkit0Name = ascCommonToolkit0Name

// Stop.ops
// # ASCCommonToolkit
// unloadComponent(controller.ascCommonToolkit0Name)

// component_controller.cpp
// In the first constructor function (has the same name as your controller)
// // ASCCommonToolkit
// this->addProperty("ascCommonToolkit0Name", ascCommonToolkit0Name);
// In the configure hook function
// // ASCCommonToolkit Service
// ascCommonToolkit0 = this->getPeer(ascCommonToolkit0Name);
// if (ascCommonToolkit0) {
// 	legStiffness0 = ascCommonToolkit0->provides("ascCommonToolkit")->getOperation("legStiffness");
// }

// component_controller.h
// // ASCCommonToolkit
// std::string ascCommonToolkit0Name;
// TaskContext *ascCommonToolkit0;
// OperationCaller<double(double, double)> legStiffness0;


// To use do something like this.
// std::tie(qmA, qmB) = polLegPos2MotorPos(ql, rl);
// TODO Finish documenting all functions

// Initialize ==================================================================
#include <asc_common_toolkit/controller_component.h>

namespace atrias {
namespace controller {


// ASCCommonToolkit ============================================================
ASCCommonToolkit::ASCCommonToolkit(std::string name):
    RTT::TaskContext(name),
    logPort(name + "_log") {
    
    // Operations --------------------------------------------------------------
    this->provides("legStiffness")->addOperation("legStiffness",&ASCCommonToolkit::legStiffness, this, ClientThread).doc("Computes the virtual leg length stiffness.");
    
    this->provides("polLegPos2MotorPos")->addOperation("polLegPos2MotorPos",&ASCCommonToolkit::polLegPos2MotorPos, this, ClientThread).doc("Converts leg polar position to motor angular positions.");
    
    this->provides("polLegVel2MotorVel")->addOperation("polLegVel2MotorVel",&ASCCommonToolkit::polLegVel2MotorVel, this, ClientThread).doc("Converts leg polar velocities to motor angular velocities.");
    
    this->provides("polMotorPos2LegPos")->addOperation("polMotorPos2LegPos",&ASCCommonToolkit::polMotorPos2LegPos, this, ClientThread).doc("Converts motor angular position to leg polar positions.");
    
    // TODO cartLegPos2MotorPos
    // TODO cartLegVel2MotorVel
    // TODO inverse of all conversions
    // TODO rad2Deg
    // TODO deg2Rad
    // TODO interp
    
	
	// Variables ---------------------------------------------------------------
	
	
	
    // Logging -----------------------------------------------------------------
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
    log(Info) << "[ASCCommonToolkit] Constructed!" << endlog();
    
} // ASCCommonToolkit


// legStiffness ================================================================
double ASCCommonToolkit::legStiffness(double r, double r0) {

	// Compute non-linear ATRIAS virtual leg length stiffness
	k = ks*(sin(acos(r)) - (acos(r) - acos(r0))*cos(acos(r)))/(2.0*l1*l2*pow(sin(acos(r)), 3));
	
	// Stuff the msg and push to ROS for logging
	//logData.header = getROSHeader();
	//logData.k = k;
	//logPort.write(logData);

	return k;
	
} // legStiffness


// polMotorPos2LegPos ==========================================================
std::tuple<double, double> ASCCommonToolkit::polMotorPos2LegPos(double qmA, double qmB) {

	// Compute leg positions
    ql = ((qmA + qmB)/2.0);
    rl = cos((qmA - qmB)/2.0);
	
	return std::make_tuple(ql, rl);
	
} // polMotorPos2LegPos


// polLegPos2MotorPos ==========================================================
std::tuple<double, double> ASCCommonToolkit::polLegPos2MotorPos(double ql, double rl) {

	// Compute motor positions
    qmA = ql - acos(rl);
    qmB = ql + acos(rl);
	
	return std::make_tuple(qmA, qmB);
	
} // polLegPos2MotorPos


// polLegVel2MotorVel ==========================================================
std::tuple<double, double> ASCCommonToolkit::polLegVel2MotorVel(double ql, double dql, double drl) {

	// Compute motor velocities
    dqmA = dql + drl/sqrt(1.0 - pow(ql, 2));
    dqmB = dql - drl/sqrt(1.0 - pow(ql, 2));
	
	return std::make_tuple(dqmA, dqmB);
	
} // polLegVel2MotorVel


// conifigureHook ==============================================================
bool ASCCommonToolkit::configureHook() {
	// Log data stuff
    RTT::TaskContext* rtOpsPeer = this->getPeer("Deployer")->getPeer("atrias_rt");
    if (rtOpsPeer) {
        getROSHeader = rtOpsPeer->provides("timestamps")->getOperation("getROSHeader");
    } else {
        log(Warning) << "[ASCCommonToolkit] Can't connect to the Deployer" << endlog();
    }
    
    log(Info) << "[ASCCommonToolkit] configured!" << endlog();
    return true;
    
} // configureHook


// startHook ===================================================================
bool ASCCommonToolkit::startHook() {
    log(Info) << "[ASCCommonToolkit] started!" << endlog();
    return true;
    
} // startHook


// updateHook ==================================================================
void ASCCommonToolkit::updateHook() {

} // updateHook


// stopHook ====================================================================
void ASCCommonToolkit::stopHook() {
    log(Info) << "[ASCCommonToolkit] stopped!" << endlog();
    
} // stopHook


// cleanupHook =================================================================
void ASCCommonToolkit::cleanupHook() {
    log(Info) << "[ASCCommonToolkit] cleaned up!" << endlog();
    
} // cleanupHook


ORO_CREATE_COMPONENT(ASCCommonToolkit)

} // namespace controller
} // namespace atrias
