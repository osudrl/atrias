#ifndef ASC_SLIP_MODEL_HPP
#define ASC_SLIP_MODEL_HPP
 
/**
  * @file ASC_SLIP_MODEL.hpp
  * @author Mikhail Jones
  * @brief This implements a numerical integrator to solve the SLIP dynamics.
  */

// The include for the controller class
#include <atrias_control_lib/AtriasController.hpp>

// And for the logging helper class
#include <atrias_control_lib/LogPort.hpp>

// Our log data
#include "asc_slip_model/controller_log_data.h"

// Datatypes
#include <atrias_shared/globals.h>
#include <robot_invariant_defs.h>
#include <atrias_shared/controller_structs.h>
#include <atrias_shared/atrias_parameters.h>

// Namespaces we're using
using namespace std;
//using namespace atrias_msgs;

// Our namespaces
namespace atrias {
namespace controller {

// The subcontroller class itself
class ASCSlipModel : public AtriasController {
	public:
		/**
		  * @brief The constructor for this subcontroller
		  * @param parent The instantiating, "parent" controller.
		  * @param name The name for this controller.
		  */                  
		ASCSlipModel(AtriasController *parent, string name);

		// SLIP model parameters
		double k, dk, r0, m;
		double h;

		/**
		  * @brief Advances the RK4 fixed timestep numerical integrator.
		  * @param slipState The current state-space parameters.
		  * @return slipState The computed next step state-space parameters.
		  */
		SlipState advanceRK4(SlipState slipState);

		// State-space
		double r, dr, q, dq;
		double rNew, drNew, qNew, dqNew;
				
		/**
		  * @brief Advances the RK5 fixed timestep numerical integrator.
		  * @param slipState The current state-space parameters.
		  * @return slipState The computed next step state-space parameters.
		  */
		SlipState advanceRK5(SlipState slipState);
						
		/**
		  * @brief Computes the leg force.
		  * @param slipState The current state-space parameters.
		  * @return legForce The computed component forces at the toe.
		  */				
		LegForce force(SlipState slipState);

		// Leg forces
		LegForce legForce;


	private:
		/** 
		  * @brief This is our logging port.
		  * You may have as many of these as you'd like of various types.
		  */
		LogPort<asc_slip_model::controller_log_data> log_out;
};

}
}

#endif // ASC_SLIP_MODEL_HPP
