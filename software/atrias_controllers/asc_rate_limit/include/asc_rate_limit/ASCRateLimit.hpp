#ifndef ASCRateLimit_HPP
#define ASCRateLimit_HPP

/**
 * @file ASCRateLimit.hpp
 * @author Ryan Van Why
 * @brief This implements a command rate limiter.
 */

// The include for the controller class
#include <atrias_control_lib/AtriasController.hpp>
// And for the logging helper class
#include <atrias_control_lib/LogPort.hpp>

// These are for the delta time calculation
#include <atrias_shared/globals.h>
#include <robot_invariant_defs.h>

// Our log data
#include "asc_rate_limit/controller_log_data.h"

// Namespaces we're using
using namespace std;
using namespace asc_rate_limit;

// Our namespaces
namespace atrias {
namespace controller {

// The subcontroller class itself
class ASCRateLimit : public AtriasController {
	public:
		/**
		  * @brief The constructor for this subcontroller
		  * @param parent The instantiating, "parent" controller.
		  * @param name   The name for this controller (such as "rateLimLA")
		  */
		ASCRateLimit(AtriasController *parent, string name);

		/**
		  * @brief The main function for this controller.
		  * @param tgt  The desired output value
		  * @param rate The maximum output rate
		  * @return A rate-limited output.
		  *
		  * We're overloading this operator to make this class a functor. This makes
		  * calling this controller more convenient.
		  * 
		  * This is a more convenient overload for when abs(posRate) = abs(negRate)
		  */
		double operator()(double tgt, double rate);

		/**
		  * @brief The main function for this controller.
		  * @param tgt     The desired output value
		  * @param posRate The maximum output rate
		  * @param negRate The minimum (negative) output rate
		  * @return A rate-limited output.
		  *
		  * We're overloading this operator to make this class a functor. This makes
		  * calling this controller more convenient.
		  */
		double operator()(double tgt, double posRate, double negRate);

		/**
		  * @brief Resets the output to a specified value.
		  * @param new_value The new output value.
		  * @return The new value, for convenience (if desired).
		  */
		double reset(double new_value);

	private:
		/** 
		  * @brief This is our logging port.
		  * You may have as many of these as you'd like of various types.
		  */
		LogPort<asc_rate_limit::controller_log_data> log_out;
};

// End namespaces
}
}

#endif // ASCRateLimit_HPP

// vim: noexpandtab
