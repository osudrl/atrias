#ifndef ATCEqPoint_HPP
#define ATCEqPoint_HPP

/**
  * @file ATCEqPoint.hpp
  * @brief This implements a kinematic constraints-based walking controller for ATRIAS
  */

// Top-level controllers are components, so we need to include this.
#include <rtt/Component.hpp>

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>
// No logging helper is needed -- one log port is automatically produced.

// Our logging data type.
#include "atc_eq_point/controller_log_data.h"
// The type transmitted from the GUI to the controller
#include "atc_eq_point/controller_input.h"
// The type transmitted from the controller to the GUI
#include "atc_eq_point/controller_status.h"

// Our subcontroller types
#include <asc_common_toolkit/ASCCommonToolkit.hpp>
#include <asc_hip_boom_kinematics/ASCHipBoomKinematics.hpp>
#include <asc_pd/ASCPD.hpp>

// Namespaces we're using
using namespace std;
using namespace atc_eq_point;

// Our namespaces
namespace atrias {
namespace controller {

/* Our class definition. We subclass ATC for a top-level controller.
 * If we don't need a data type (such as the controller-to-gui message),
 * we simply leave that spot in the template blank. The following example
 * shows the necessary definition if this controller were not to transmit
 * data to the GUI:
 *     class ATCEqPoint : public ATC<log_data, gui_to_controller,>
 *
 * Here, we don't need any log data, but we do communicate both ways w/ the GUI
 */
class ATCEqPoint : public ATC<controller_log_data_, controller_input_, controller_status_> {
	public:
		/**
		  * @brief The constructor for this controller.
		  * @param name The name of this component.
		  * Every top-level controller will have this name parameter,
		  * just like current controllers.
		  */
		ATCEqPoint(string name);
	
	private:
		/**
		  * @brief This is the main function for the top-level controller.
		  * The ATC class automatically handles startup and shutdown,
		  * if they are not disabled.
		  */
		void controller();

		// Common Toolkit
		ASCCommonToolkit commonToolkit;

		// Hip kinematics control ASC
		ASCHipBoomKinematics hipKine;

		// PD controllers for each motor
		ASCPD pd0Controller; // Left  A
		ASCPD pd1Controller; // Left  B
		ASCPD pd2Controller; // Left  Hip
		ASCPD pd3Controller; // Right A
		ASCPD pd4Controller; // Right B
		ASCPD pd5Controller; // Right Hip

		// PD gains for convenience (these are not set by the GUI)
		double legP, legD, hipP, hipD;
		
		// internal variables
		uint8_t state;
		bool    sw_stance;
		bool    sw_flight;
		double  l_rLeg;
		double  phi_rLeg;
		double  l_lLeg;
		double  phi_lLeg;
		double  s;
		double  t;
		double  l_swing;
		bool    rGC;
		bool    lGC;
		double  amp;
		double  phi_MsA;
		double  phi_MfB;
		double  phiAf_des;
		double  phiBs_des;
		double  max_phi_swing;
		double  time;

		// Whether we are in idle mode (so the robot can't re-enable)
		bool idle_mode;
};

}
}

#endif // ATCEqPoint_HPP

// vim: noexpandtab
