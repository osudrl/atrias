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
// - OperationCaller<SlipConditions(SlipConditions)> slipAdvanceTimeStep;
// - OperationCaller<LegForce(SlipConditions)> slipConditionsToForce;

// To use do something like this.
// - slipConditions = slipAdvanceTimeStep(slipConditions);
// - legForce = slipConditionsToForce(slipConditions);

#include <asc_slip_model_solver/asc_slip_model_solver-service.h>

namespace atrias {
namespace controller {

// Constructor
ASCSlipModelSolver::ASCSlipModelSolver(TaskContext* owner):Service("ascSlipModelSolver", owner) {

    this->addOperation("slipAdvanceTimeStep", &ASCSlipModelSolver::slipAdvanceTimeStep, this).doc("Given a set of initial conditions, returns 4th order Runge-Kutta numerical approximation of next time step.");
	this->addOperation("slipConditionsToForce", &ASCSlipModelSolver::slipConditionsToForce, this).doc("Given a set of initial conditions, returns X-Z component forces.");

}

SlipConditions ASCSlipModelSolver::slipAdvanceTimeStep(SlipConditions slipConditions) {

	// Advance time step.
	slipConditions.rOld = slipConditions.r;
	slipConditions.drOld = slipConditions.dr;
	slipConditions.qOld = slipConditions.q;
	slipConditions.dqOld = slipConditions.dq;

	// Time step.
	delta = 0.001;

	// Robot parameters.
	g = -9.81;
	ks = 16000.0;
	m = 60.0;
	r0 = 0.85;

	if (slipConditions.r > r0) {
		// Do nothing because we are not in stance.

	} else {
		// Advance to next timestep because we are in stance.
		// SLIP model 4th order Runge-Kutta numerical solution.
		slipConditions.r = slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)/2.0)/3.0 + delta*slipConditions.drOld/6.0 + delta*(slipConditions.drOld + delta*(g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - ks*(slipConditions.rOld - r0 + delta*slipConditions.drOld/2.0)/m)/2.0)/3.0 + delta*(slipConditions.drOld + delta*(-g*sin(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)/2.0)/2.0)*pow(slipConditions.dqOld - delta*(g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld), 2) - ks*(slipConditions.rOld - r0 + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)/2.0)/2.0)/m))/6.0;

		slipConditions.drOld = slipConditions.drOld + delta*(g*sin(slipConditions.qOld + delta*(slipConditions.dqOld - delta*(g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld))) + (slipConditions.rOld + delta*(slipConditions.drOld + delta*(g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - ks*(slipConditions.rOld - r0 + delta*slipConditions.drOld/2.0)/m)/2.0))*pow(slipConditions.dqOld - delta*(g*cos(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (2.0*slipConditions.drOld + delta*(g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - ks*(slipConditions.rOld - r0 + delta*slipConditions.drOld/2.0)/m))*(slipConditions.dqOld - delta*(g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld)))/(slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)/2.0)/2.0), 2) - ks*(slipConditions.rOld - r0 + delta*(slipConditions.drOld + delta*(g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - ks*(slipConditions.rOld - r0 + delta*slipConditions.drOld/2.0)/m)/2.0))/m)/6.0 + delta*(-g*sin(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)/2.0)/2.0)*pow(slipConditions.dqOld - delta*(g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld), 2) - ks*(slipConditions.rOld - r0 + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)/2.0)/2.0)/m)/3.0 + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)/6.0 + delta*(g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - ks*(slipConditions.rOld - r0 + delta*slipConditions.drOld/2.0)/m)/3.0;

		slipConditions.qOld = slipConditions.qOld + delta*slipConditions.dqOld/6.0 + delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/3.0 + delta*(slipConditions.dqOld - delta*(g*cos(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (2.0*slipConditions.drOld + delta*(g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - ks*(slipConditions.rOld - r0 + delta*slipConditions.drOld/2.0)/m))*(slipConditions.dqOld - delta*(g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld)))/(slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)/2.0)/2.0))/6.0 + delta*(slipConditions.dqOld - delta*(g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*(2.0*slipConditions.dqOld*slipConditions.drOld + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*(2.0*slipConditions.drOld + delta*(slipConditions.rOld*slipConditions.dqOld*slipConditions.dqOld + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld))/3.0;

		slipConditions.dqOld = slipConditions.dqOld - delta*(g*cos(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + ((2.0*slipConditions.drOld) + delta*(g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - ks*(slipConditions.rOld - r0 + delta*slipConditions.drOld/2.0)/m))*(slipConditions.dqOld - delta*(g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld)))/(3.0*slipConditions.rOld + 3.0/2.0*delta*(slipConditions.drOld + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)/2.0)) - delta*(g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)))/(3.0*slipConditions.rOld + 3.0/2.0*delta*slipConditions.drOld) - delta*(g*cos(slipConditions.qOld + delta*(slipConditions.dqOld - delta*(g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld))) + ((2.0*slipConditions.drOld) + 2.0*delta*(-g*sin(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + (slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)/2.0)/2.0)*pow(slipConditions.dqOld - delta*(g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld), 2) - ks*(slipConditions.rOld - r0 + delta*(slipConditions.drOld + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)/2.0)/2.0)/m))*(slipConditions.dqOld - delta*(g*cos(-slipConditions.qOld - delta*(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)/2.0) + ((2.0*slipConditions.drOld) + delta*(g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - ks*(slipConditions.rOld - r0 + delta*slipConditions.drOld/2.0)/m))*(slipConditions.dqOld - delta*(g*cos(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + (slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0)*((2.0*slipConditions.drOld) + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)))/(2.0*slipConditions.rOld + delta*slipConditions.drOld)))/(slipConditions.rOld + delta*(slipConditions.drOld + delta*(slipConditions.rOld*(slipConditions.dqOld*slipConditions.dqOld) + g*sin(slipConditions.qOld) + ks*(r0 - slipConditions.rOld)/m)/2.0)/2.0)))/(6.0*slipConditions.rOld + 6.0*delta*(slipConditions.drOld + delta*(g*sin(slipConditions.qOld + delta*slipConditions.dqOld/2.0) + pow(slipConditions.dqOld - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + g*cos(slipConditions.qOld))/slipConditions.rOld/2.0, 2)*(slipConditions.rOld + delta*slipConditions.drOld/2.0) - ks*(slipConditions.rOld - r0 + delta*slipConditions.drOld/2.0)/m)/2.0)) - delta*((2.0*slipConditions.dqOld*slipConditions.drOld) + g*cos(slipConditions.qOld))/slipConditions.rOld/6.0;

	}

    return slipConditions;

} // slipAdvanceTimeStep


// slipConditionsToForce =======================================================
LegForce ASCSlipModelSolver::slipConditionsToForce(SlipConditions slipConditions) {

	// Robot parameters.
	g = -9.81;
	ks = 16000.0;
	m = 60.0;
	r0 = 0.85;

	// If virtual SLIP model is in flight.
	if (slipConditions.r > r0) {
		// Define component force.
		legForce.fx = 0.0;
		legForce.fz = 0.0;

	// If virtual SLIP model is in NOT in flight (is in stance).
	} else {
		// Define component forces.
		legForce.fx = ks*(slipConditions.r - r0)*cos(slipConditions.q);
		legForce.fz = ks*(slipConditions.r - r0)*sin(slipConditions.q);
		legForce.dfx = ks*(slipConditions.dr - 0.0)*cos(slipConditions.dq);
		legForce.dfz = ks*(slipConditions.dr - 0.0)*sin(slipConditions.dq);
	}

	return legForce;

} // slipConditionsToForce

ORO_SERVICE_NAMED_PLUGIN(ASCSlipModelSolver, "ascSlipModelSolver")

} // namespace controller
} // namespace atrias
