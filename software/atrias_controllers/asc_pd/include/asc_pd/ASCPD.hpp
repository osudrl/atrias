#ifndef ASCPD_HPP
#define ASCPD_HPP

/**
 * @file ASCPD.hpp
 * @author Ryan Van Why
 * @brief This implements a simple PD controller.
 */

// The include for the controller class
#include <atrias_control_lib/AtriasController.hpp>
// And for the logging helper class
#include <atrias_control_lib/LogPort.hpp>

// Our log data
#include "asc_pd/controller_log_data.h"

// Namespaces we're using
using namespace std;

// Our namespaces
namespace atrias {
namespace controller {

// The subcontroller class itself
class ASCPD : public AtriasController {
	public:
		/**
		  * @brief The constructor for this subcontroller
		  * @param parent The instantiating, "parent" controller.
		  * @param name   The name for this controller (such as "pdLeftA")
		  */
		ASCPD(AtriasController *parent, string name);

		/**
		  * @brief The main function for this controller.
		  * @param desPos The desired process value
		  * @param curPos The current process value
		  * @param desVel The desired rate of change in process value
		  * @param curVel The current rate of change of process value
		  * @return A commanded output.
		  *
		  * We're overloading this operator to make this class a functor. This makes
		  * calling this controller more convenient.
		  */
		double operator()(double desPos, double curPos, double desVel, double curVel);

		/**
		  * @brief Our P gain.
		  */
		double P;

		/**
		  * @brief Our D gain.
		  */
		double D;

	private:
		/** 
		  * @brief This is our logging port.
		  * You may have as many of these as you'd like of various types.
		  */
		LogPort<asc_pd::controller_log_data> log_out;
};

// End namespaces
}
}

#endif // ASCPD_HPP

// vim: noexpandtab
