#ifndef ATC_SLIP_HOPPING_HPP
#define ATC_SLIP_HOPPING_HPP

/**
  * @file ATC_SLIP_HOPPING.hpp
  * @author Mikhail Jones
  * @brief This implements a SLIP based template controller.
  */

// Top-level controllers are components, so we need to include this.
#include <rtt/Component.hpp>

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>
// No logging helper is needed -- one log port is automatically produced.

// Our logging data type.
#include "atc_slip_hopping/controller_log_data.h"

// The type transmitted from the GUI to the controller
#include "atc_slip_hopping/controller_input.h"

// The type transmitted from the controller to the GUI
#include "atc_slip_hopping/controller_status.h"

// Our subcontroller types
#include <asc_common_toolkit/ASCCommonToolkit.hpp>
#include <asc_slip_model/ASCSlipModel.hpp>
#include <asc_leg_force/ASCLegForce.hpp>
#include <asc_hip_boom_kinematics/ASCHipBoomKinematics.hpp>

// Datatypes
#include <atrias_msgs/robot_state.h>
#include <atrias_shared/controller_structs.h>
#include <atrias_shared/atrias_parameters.h>

// Namespaces we're using
using namespace std;
using namespace atc_slip_hopping;

// Our namespaces
namespace atrias {
namespace controller {

/* Our class definition. We subclass ATC for a top-level controller.
 * If we don't need a data type (such as the controller-to-gui message),
 * we simply leave that spot in the template blank. The following example
 * shows the necessary definition if this controller were not to transmit
 * data to the GUI:
 *     class ATC : public ATC<log_data, gui_to_controller,>
 *
 * Here, we don't need any log data, but we do communicate both ways w/ the GUI
 */
class ATCSlipHopping : public ATC<atc_slip_hopping::controller_log_data, controller_input, controller_status> {
	public:
		/**
		  * @brief The constructor for this controller.
		  * @param name The name of this component.
		  * Every top-level controller will have this name parameter,
		  * just like current controllers.
		  */
		ATCSlipHopping(string name);
	
	private:
		/**
		  * @brief This is the main function for the top-level controller.
		  * The ATC class automatically handles startup and shutdown,
		  * if they are not disabled.
		  */
		void controller();
		
		// States
		int controllerState, runningState;
		double k;
	
	
		/**
		  * @brief These are function within the top-level controller.
		  */	
		void passiveStanceControl();
		void forceStanceControl();
		void egbFlightControl();
		
		/**
		  * @brief These are sub controllers used by the top level controller.
		  */
		ASCCommonToolkit ascCommonToolkit;
		ASCSlipModel ascSlipModel;
		ASCLegForce ascLegForce;
		ASCHipBoomKinematics ascHipBoomKinematics;

};

}
}

#endif // ATC_SLIP_HOPPING_HPP
