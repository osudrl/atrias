#ifndef ATCMP_HPP
#define ATCMP_HPP

/**
  * @file ATCMP.hpp
  * @author Ryan Van Why
  * @brief This implements motor position control for every motor
  */

// Include the ATC class
#include <atrias_lib_control/ATC.hpp>
// No logging helper is needed -- one log port is automatically produced.

// Our logging data type.
#include "atc_motor_position/controller_log_data.h"
// The type transmitted from the GUI to the controller
#include "atc_motor_position/gui_to_controller.h"
// The type transmitted from the controller to the GUI
#include "atc_motor_position/controller_to_gui.h"

// Our subcontroller types
#include <asc_pd/ASCPD.hpp>
#include <asc_rate_limit/ASCRateLimit.hpp>

// Namespaces we're using
using namespace std;
using namespace atrias_msgs;

// Our namespaces
namespace atrias {
namespace controller {

/* Our class definition. We subclass ATC for a top-level controller.
 * If we don't need a data type (such as the controller-to-gui message),
 * we simply leave that spot in the template blank. The following example
 * shows the necessary definition if this controller were not to transmit
 * data to the GUI:
 *     class ATCMP : public ATC<log_data, gui_to_controller,>
 *
 * Here, we don't need any log data, but we do communicate both ways w/ the GUI
 */
class ATCMP : public ATC<controller_log_data, gui_to_controller, controller_to_gui> {
	public:
		/**
		  * @brief The constructor for this controller.
		  * @param name The name of this component.
		  * Every top-level controller will have this name parameter,
		  * just like current controllers.
		  */
		ATCMP(string name);
	
	private:
		/**
		  * @brief This is the main function for the top-level controller.
		  * The ATC class automatically handles startup and shutdown,
		  * if they are not disabled.
		  */
		void controller();

		// PD controllers for each motor
		ASCPD pdLA; // Left  A
		ASCPD pdLB; // Left  B
		ASCPD pdLH; // Left  Hip
		ASCPD pdRA; // Right A
		ASCPD pdRB; // Right B
		ASCPD pdRH; // Right Hip

		// Rate limiters for each motor
		ASCRateLimit rateLimLA // Left  A
		ASCRateLimit rateLimLB // Left  B
		ASCRateLimit rateLimLH // Left  Hip
		ASCRateLimit rateLimRA // Right A
		ASCRateLimit rateLimRB // Right B
		ASCRateLimit rateLimRH // Right Hip
};

}
}

#endif // ATCMP_HPP

// vim: noexpandtab
