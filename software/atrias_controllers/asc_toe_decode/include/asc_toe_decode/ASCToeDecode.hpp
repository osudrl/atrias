#ifndef ASCToeDecode_HPP
#define ASCToeDecode_HPP

/**
 * @file ASCToeDecode.hpp
 * @author Ryan Van Why
 * @brief This subcontroller reads the toe force data and deduces whether or not it is contacting the ground.
 */

// The include for the controller class
#include <atrias_control_lib/AtriasController.hpp>
// And for the logging helper class
#include <atrias_control_lib/LogPort.hpp>

// Our log data
#include "asc_toe_decode/controller_log_data.h"

// Namespaces we're using
using namespace std;

// Our namespaces
namespace atrias {
namespace controller {

// The subcontroller class itself
class ASCToeDecode : public AtriasController {
	public:
		/**
		  * @brief The constructor for this subcontroller
		  * @param parent The instantiating, "parent" controller.
		  * @param name   The name for this controller (such as "pdLeftA")
		  */
		ASCToeDecode(AtriasController *parent, string name);

		/**
		  * @brief The main function for this controller.
		  * @param force The current toe force reading.
		  * @return Whether or not the toe is on the ground
		  *
		  * Call this function every millisecond with new data.
		  */
		double operator()(uint16_t force);

		/**
		  * @brief This returns true if the toe is contacting the ground.
		  * This value is not updated until the main function is called,
		  * so this should not b called until after that function.
		  * This is only for convenience -- it returns the same thing as
		  * the main operator() function.
		  * @return True if the toe is touching the ground, false otherwise.
		  */
		bool onGround();

		/**
		  * @brief The gain for our filter.
		  * This has not been tuned yet, but once it has been, it will have
		  * a good default
		  */
		double filter_gain;

		/**
		  * @brief The threshold for detection.
		  * This has not been tuned yet, but once tuning is complete, it will have a good default.
		  */
		double threshold;

	private:
		/** 
		  * @brief This is our logging port.
		  */
		LogPort<asc_toe_decode::controller_log_data_> log_out;
};

// End namespaces
}
}

#endif // ASCToeDecode_HPP

// vim: noexpandtab
