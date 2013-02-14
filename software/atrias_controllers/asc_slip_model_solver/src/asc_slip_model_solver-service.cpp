// Title: ASCSlipModelSolver
// Description: Defines and solves SLIP model conditions and forces.
// Author: Mikhail Jones

// Usage: To use this subcontroller, add the following pieces of code to the the designated files in your controller folder.

// Manifest.xml
// - <depend package="asc_slip_model_solver"/>

// Start.ops
// - # ASCSlipModelSolver
// - require("ascSlipModelSolver")
// - loadService("controller", "ascSlipModelSolver")

// component_controller.cpp
// - // ASCSlipModelSolver Service
// - slipAdvanceTimeStep = this->provides("ascSlipModelSolver")->getOperation("slipAdvanceTimeStep");
// - slipConditionsToForce = this->provides("ascSlipModelSolver")->getOperation("slipConditionsToForce");

// component_controller.h
// - // ASCSlipModelSolver
// - OperationCaller<SlipConditions(SlipModel, SlipConditions)> slipAdvanceTimeStep;
// - OperationCaller<LegForce(SlipModel, SlipConditions)> slipConditionsToForce;

// To use do something like this.
// - slipConditions = slipAdvanceTimeStep(slipModel, slipConditions);
// - legForce = slipConditionsToForce(slipModel, slipConditions);

// TODO - Add error catch incase structs are empty.

#include <asc_slip_model_solver/asc_slip_model_solver-service.h>

namespace atrias {
namespace controller {


//ASCSlipModelSolver ===========================================================
ASCSlipModelSolver::ASCSlipModelSolver(TaskContext* owner):Service("ascSlipModelSolver", owner) {

    this->addOperation("slipAdvanceTimeStep", &ASCSlipModelSolver::slipAdvanceTimeStep, this).doc("Given a set of initial conditions, returns 4th order Runge-Kutta numerical approximation of next time step.");
    
	this->addOperation("slipConditionsToForce", &ASCSlipModelSolver::slipConditionsToForce, this).doc("Given a set of initial conditions, returns X-Z component forces.");

}


// slipAdvanceTimeStep =========================================================
SlipConditions ASCSlipModelSolver::slipAdvanceTimeStep(SlipModel slipModel, SlipConditions slipConditions) {

	// Example slipModel struct.
	//slipModel.g = -9.81;
	//slipModel.ks = 16000.0;
	//slipModel.m = 60.0;
	//slipModel.r0 = 0.85;
	
	// Advance time step.
	slipConditions.rOld = slipConditions.r;
	slipConditions.drOld = slipConditions.dr;
	slipConditions.qOld = slipConditions.q;
	slipConditions.dqOld = slipConditions.dq;

	// Time step.
	delta = 0.001;

	if (slipConditions.r > slipModel.r0) {
		// Do nothing because we are not in stance.
		printf("SLIP: DO NOTHING!\n");
		slipConditions.isFlight = true;
		
	} else {
		// We are in stance.
		printf("SLIP: NEXT TIME STEP!\n");
		slipConditions.isFlight = false;
		
		// Advance to next timestep because we are in stance.
		// SLIP model 4th order Runge-Kutta numerical solution.
		slipConditions.r = slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/3.0 + delta*slipConditions.drOld/6.0 + delta*(slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m)/2.0)/3.0 + delta*(slipConditions.drOld + delta*(-slipModel.g*sin(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)*pow(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld), 2) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)/slipModel.m))/6.0;

		slipConditions.dr = slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld))) + (slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m)/2.0))*pow(slipConditions.dqOld - delta*(slipModel.g*cos(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (2.0*slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m))*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld)))/(slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0), 2) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*(slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m)/2.0))/slipModel.m)/6.0 + delta*(-slipModel.g*sin(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)*pow(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld), 2) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)/slipModel.m)/3.0 + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/6.0 + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m)/3.0;

		slipConditions.q = slipConditions.qOld + delta*slipConditions.dqOld/6.0 + delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/3.0 + delta*(slipConditions.dqOld - delta*(slipModel.g*cos(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (2.0*slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m))*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld)))/(slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0))/6.0 + delta*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld))/3.0;

		slipConditions.dq = slipConditions.dqOld - delta*(slipModel.g*cos(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + ((2.0*slipConditions.drOld) + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m))*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld)))/(3.0*slipConditions.rOld + 3.0/2.0*delta*(slipConditions.drOld + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)) - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(3.0*slipConditions.rOld + 3.0/2.0*delta*slipConditions.drOld) - delta*(slipModel.g*cos(slipConditions.qOld + delta*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld))) + ((2.0*slipConditions.drOld) + 2.0*delta*(-slipModel.g*sin(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)*pow(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld), 2) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*(slipConditions.drOld + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)/slipModel.m))*(slipConditions.dqOld - delta*(slipModel.g*cos(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + ((2.0*slipConditions.drOld) + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m))*(slipConditions.dqOld - delta*(slipModel.g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld)))/(slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + slipModel.g*sin(slipConditions.qOld) + slipModel.ks*(slipModel.r0 - slipConditions.rOld)/slipModel.m)/2.0)/2.0)))/(6.0*slipConditions.rOld + 6.0*delta*(slipConditions.drOld + delta*(slipModel.g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - slipModel.ks*(slipConditions.rOld - slipModel.r0 + delta*slipConditions.drOld/2.0)/slipModel.m)/2.0)) - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + slipModel.g*cos(slipConditions.qOld))/slipConditions.rOld/6.0;

	}
	
	// Debug statements.
	//printf("r: %f\n", slipConditions.r);
	//printf("dr: %f\n", slipConditions.dr);
	//printf("q: %f\n", slipConditions.q);
	//printf("dq: %f\n", slipConditions.dq);

    return slipConditions;

} // slipAdvanceTimeStep


// slipConditionsToForce =======================================================
LegForce ASCSlipModelSolver::slipConditionsToForce(SlipModel slipModel, SlipConditions slipConditions) {

	// If virtual SLIP model is in flight.
	if (slipConditions.isFlight) {
		// Define component force.
		legForce.fx = 0.0;
		legForce.fz = 0.0;
		legForce.dfx = 0.0;
		legForce.dfz = 0.0;

	// If virtual SLIP model is in NOT in flight (is in stance).
	} else {
		// Define component forces.
		legForce.fx = slipModel.ks*(slipConditions.r - slipModel.r0)*cos(slipConditions.q);
		legForce.fz = slipModel.ks*(slipConditions.r - slipModel.r0)*sin(slipConditions.q);
		legForce.dfx = slipModel.ks*(slipConditions.dr - 0.0)*cos(slipConditions.q);
		legForce.dfz = slipModel.ks*(slipConditions.dr - 0.0)*sin(slipConditions.q);
	}

	// Debug statements.
	//printf("fx: %f\n", legForce.fx);
	//printf("fz: %f\n", legForce.fz);
	//printf("dfx: %f\n", legForce.dfx);
	//printf("dfz: %f\n", legForce.dfz);

	return legForce;

} // slipConditionsToForce

ORO_SERVICE_NAMED_PLUGIN(ASCSlipModelSolver, "ascSlipModelSolver")

} // namespace controller
} // namespace atrias
