// Title: ASCSlipModel
// Description: Various function related to the SLIP model.
// Author: Mikhail Jones

// Usage: To use this subcontroller, add the following pieces of code to the the designated files in your controller folder.

// Manifest.xml
// <depend package="asc_slip_model"/>

// Start.ops
// # ASCSlipModel
// import("asc_slip_model")
// var string ascSlipModel0Name = atrias_cm.getUniqueName("controller", "ascSlipModel")
// loadComponent(ascSlipModel0Name, "ASCSlipModel")
// addPeer("controller", ascSlipModel0Name)
// connectPeers("Deployer", ascSlipModel0Name)
// controller.ascSlipModel0Name = ascSlipModel0Name

// Stop.ops
// # ASCSlipModel
// unloadComponent(controller.ascSlipModel0Name)

// component_controller.cpp
// In the first constructor function (has the same name as your controller)
// // ASCSlipModel
// this->addProperty("ascSlipModel0Name", ascSlipModel0Name);
// In the configure hook function
// // ASCSlipModel Service
// ascSlipModel0 = this->getPeer(ascSlipModel0Name);
// if (ascSlipModel0) {
// 	slipAdvance0 = ascSlipModel0->provides("ascSlipModel")->getOperation("slipAdvance");
// 	slipForce0 = ascSlipModel0->provides("ascSlipModel")->getOperation("slipAdvance");
// }

// component_controller.h
// // ASCSlipModel
// std::string ascSlipModel0Name;
// TaskContext *ascSlipModel0;
// OperationCaller<SlipConditions(SlipModel, SlipConditions)> slipAdvance0;
// OperationCaller<LegForce(SlipModel, SlipConditions)> slipForce0;

// To use do something like this.
// // Define slipModel struct.
// slipModel.g = -9.81;
// slipModel.ks = 16000.0;
// slipModel.m = 60.0;
// slipModel.r0 = 0.85;
//
// // Define slipConditions struct.
// slipConditions.r = 0.85;
// slipConditions.dr = -sqrt(2.0*9.81*0.05);
// slipConditions.q = M_PI/2.0;
// slipConditions.dq = 0.0;
//
// // Compute and set legForce.
// slipConditions = slipAdvance0(slipModel, slipConditions);
// legForce = slipForce0(slipModel, slipConditions);

// TODO - Add error catch incase structs are empty.


#include <asc_slip_model/controller_component.h>

namespace atrias {
namespace controller {


//ASCSlipModel =================================================================
ASCSlipModel::ASCSlipModel(std::string name):
	RTT::TaskContext(name),
	logPort(name + "_log") {

	// Add operations
	this->provides("ascSlipModel")->addOperation("slipAdvance", &ASCSlipModel::slipAdvance, this).doc("Given a set of initial conditions, returns 4th order Runge-Kutta numerical approximation of next time step.");
    
	this->provides("ascSlipModel")->addOperation("slipForce", &ASCSlipModel::slipForce, this).doc("Given a set of SLIP model conditions, returns X-Z component forces.");
    
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

    log(Info) << "[ASCSlipModel] Constructed!" << endlog();
}


// slipAdvance =================================================================
SlipConditions ASCSlipModel::slipAdvance(SlipModel slipModel, SlipConditions slipConditions) {

	// Advance time step.
	slipConditions.rOld = slipConditions.r;
	slipConditions.drOld = slipConditions.dr;
	slipConditions.qOld = slipConditions.q;
	slipConditions.dqOld = slipConditions.dq;

	// Time step.
	delta = 0.001;

	if (slipConditions.r > slipModel.r0) {
		// Do nothing because we are not in stance.
		slipConditions.isFlight = true;
		
	} else {
		// We are in stance.
		slipConditions.isFlight = false;
		
		// Nonlinear ATRIAS spring constant dependent on leg length.
		//slipModel.ks = 2.0*slipModel.ks/sqrt(1.0 - pow(slipConditions.r, 2.0));
		
		// Advance to next timestep because we are in stance.
		// SLIP model 4th order Runge-Kutta numerical solution.
		slipConditions.r = slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/3.0 + delta*slipConditions.drOld/6.0 + delta*(slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m)/2.0)/3.0 + delta*(slipConditions.drOld + delta*(-slipModel.g*sin(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)*pow(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld), 2) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)/slipModel.m))/6.0;

		slipConditions.dr = slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld))) + (slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m)/2.0))*pow(slipConditions.dqOld - delta*(slipModel.g*cos(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (2.0*slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m))*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld)))/(slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0), 2) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*(slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m)/2.0))/slipModel.m)/6.0 + delta*(-slipModel.g*sin(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)*pow(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld), 2) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)/slipModel.m)/3.0 + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/6.0 + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m)/3.0;

		slipConditions.q = slipConditions.qOld + delta*slipConditions.dqOld/6.0 + delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/3.0 + delta*(slipConditions.dqOld - delta*(slipModel.g*cos(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (2.0*slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m))*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld)))/(slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0))/6.0 + delta*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld))/3.0;

		slipConditions.dq = slipConditions.dqOld - delta*(slipModel.g*cos(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + ((2.0*slipConditions.drOld) + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m))*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld)))/(3.0*slipConditions.rOld + 3.0/2.0*delta*(slipConditions.drOld + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)) - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(3.0*slipConditions.rOld + 3.0/2.0*delta*slipConditions.drOld) - delta*(slipModel.g*cos(slipConditions.qOld + delta*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld))) + ((2.0*slipConditions.drOld) + 2.0*delta*(-slipModel.g*sin(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)*pow(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld), 2) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*(slipConditions.drOld + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)/slipModel.m))*(slipConditions.dqOld - delta*(slipModel.g*cos(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + ((2.0*slipConditions.drOld) + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m))*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld)))/(slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)))/(6.0*slipConditions.rOld + 6.0*delta*(slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m)/2.0)) - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/6.0;

	}
	
	// DEBUG STATEMENTS (Not real-time safe).
	//printf("r: %f\n", slipConditions.r);
	//printf("dr: %f\n", slipConditions.dr);
	//printf("q: %f\n", slipConditions.q);
	//printf("dq: %f\n", slipConditions.dq);

    // Stuff the msg and push to ROS for logging
    logData.r = slipConditions.r;
    logData.dr = slipConditions.dr;
    logData.q = slipConditions.q;
    logData.dq = slipConditions.dq;
    logPort.write(logData);

    return slipConditions;

} // slipAdvance


// slipForce ===================================================================
LegForce ASCSlipModel::slipForce(SlipModel slipModel, SlipConditions slipConditions) {

	// If virtual SLIP model is in flight.
	if (slipConditions.isFlight) {
		// Define component force.
		legForce.fx = 0.0;
		legForce.fz = 0.0;
		legForce.dfx = 0.0;
		legForce.dfz = 0.0;

	// If virtual SLIP model is in NOT in flight (is in stance).
	} else {
		// Nonlinear ATRIAS spring constant dependent on leg length.
		//slipModel.ks = 2.0*slipModel.ks/sqrt(1.0 - pow(slipConditions.r, 2.0));
	
		// Define component forces.
		legForce.fx = slipModel.ks*(slipConditions.r - slipModel.r0)*cos(slipConditions.q);
		legForce.fz = slipModel.ks*(slipConditions.r - slipModel.r0)*sin(slipConditions.q);
		legForce.dfx = slipModel.ks*(slipConditions.dr - 0.0)*cos(slipConditions.q);
		legForce.dfz = slipModel.ks*(slipConditions.dr - 0.0)*sin(slipConditions.q);
	}

	// DEBUG STATEMENTS (Not real-time safe).
	//printf("fx: %f\n", legForce.fx);
	//printf("fz: %f\n", legForce.fz);
	//printf("dfx: %f\n", legForce.dfx);
	//printf("dfz: %f\n", legForce.dfz);

    // Stuff the msg and push to ROS for logging
    logData.fx = legForce.fx;
    logData.fz = legForce.fz;
    logData.dfx = legForce.dfx;
    logData.dfz = legForce.dfz;
    logPort.write(logData);
    
	return legForce;

} // slipForce


// configureHook ===============================================================
bool ASCSlipModel::configureHook() {
	// Log data stuff
    RTT::TaskContext* rtOpsPeer = this->getPeer("Deployer")->getPeer("atrias_rt");
    if (rtOpsPeer) {
        getROSHeader = rtOpsPeer->provides("timestamps")->getOperation("getROSHeader");
    } else {
        log(Warning) << "[ASCSlipModel] Can't connect to the Deployer" << endlog();
    }
    
    log(Info) << "[ASCSlipModel] configured!" << endlog();
    return true;
} // configure Hook


// startHook ===================================================================
bool ASCSlipModel::startHook() {
    log(Info) << "[ASCSlipModel] started!" << endlog();
    return true;
} // startHook


// updateHook ==================================================================
void ASCSlipModel::updateHook() {
} // updateHook


// stopHook ====================================================================
void ASCSlipModel::stopHook() {
    log(Info) << "[ASCSlipModel] stopped!" << endlog();
} // stopHook


// cleanupHook =================================================================
void ASCSlipModel::cleanupHook() {
    log(Info) << "[ASCSlipModel] cleaned up!" << endlog();
} // cleanupHook


ORO_CREATE_COMPONENT(ASCSlipModel)

} // namespace controller
} // namespace atrias
