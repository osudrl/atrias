#ifndef ATCMP_HPP
#define ATCMP_HPP

/**
 * @file ATCMP.hpp
 * @author Ryan Van Why
 * @brief This implements motor position control for every motor
 */

// Top-level controllers are components, so we need to include this.
#include <rtt/Component.hpp>

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>
// No logging helper is needed -- one log port is automatically produced.

// Our logging data type.
#include "atc_motor_position/controller_log_data.h"
// The type transmitted from the GUI to the controller
#include "atc_motor_position/controller_input.h"
// The type transmitted from the controller to the GUI
#include "atc_motor_position/controller_status.h"

// Our subcontroller types
#include <asc_pd/ASCPD.hpp>
#include <asc_rate_limit/ASCRateLimit.hpp>

// Namespaces we're using
using namespace std;
using namespace atc_motor_position;

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
 */
class ATCMP : public ATC<atc_motor_position::controller_log_data, controller_input, controller_status> {
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
		 */
		void controller();

		// Include subcontrollers and variables here
		ASCPD example;
};

}
}

#endif // ATCMP_HPP
