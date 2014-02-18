#ifndef ATCDeadbeatControl_HPP
#define ATCDeadbeatControl_HPP

/**
 * @file ATCDeadbeatControl.hpp
 * @author Ryan Van Why
 * @brief To control the robot from one state to another state.
 */

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>

// Our logging data type.
#include "atc_deadbeat_control/controller_log_data.h"
// The type transmitted from the GUI to the controller
#include "atc_deadbeat_control/controller_input.h"
// The type transmitted from the controller to the GUI
#include "atc_deadbeat_control/controller_status.h"

// Include subcontrollers here

// Namespaces we're using
using namespace std;

// Our namespaces
namespace atrias {
namespace controller {

class ATCDeadbeatControl : public ATC<
	atc_deadbeat_control::controller_log_data_,
	atc_deadbeat_control::controller_input_,
	atc_deadbeat_control::controller_status_>
{
	public:
		/** 
		 * @brief The constructor for this controller.
		 * @param name The name of this component.
		 * Every top-level controller will have this name parameter,
		 * just like current controllers.
		 */
		ATCDeadbeatControl(string name);

	private:
		/** 
		 * @brief This is the main function for the top-level controller.
		 */
		void controller();

		// Include subcontrollers and variables here
};

}
}

#endif // ATCDeadbeatControl_HPP
