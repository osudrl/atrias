#ifndef ATC_SLIP_RUNNING_HPP
#define ATC_SLIP_RUNNING_HPP

/**
  * @file ATC_SLIP_RUNNING.hpp
  * @author Mikhail Jones
  * @brief This implements a SLIP based template controller.
  */

// Top-level controllers are components, so we need to include this.
#include <rtt/Component.hpp>

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>
// No logging helper is needed -- one log port is automatically produced.

// Our logging data type.
#include "atc_slip_running/controller_log_data.h"

// The type transmitted from the GUI to the controller
#include "atc_slip_running/controller_input.h"

// The type transmitted from the controller to the GUI
#include "atc_slip_running/controller_status.h"

// Our subcontroller types
#include <asc_common_toolkit/ASCCommonToolkit.hpp>
#include <asc_interpolation/ASCInterpolation.hpp>
#include <asc_slip_model/ASCSlipModel.hpp>
#include <asc_leg_force/ASCLegForce.hpp>
#include <asc_hip_boom_kinematics/ASCHipBoomKinematics.hpp>
#include <asc_pd/ASCPD.hpp>

// Datatypes
#include <robot_invariant_defs.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_shared/controller_structs.h>
#include <atrias_shared/atrias_parameters.h>

// Namespaces we're using
using namespace std;
using namespace atc_slip_running;

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
class ATCSlipRunning : public ATC<atc_slip_running::controller_log_data_, controller_input_, controller_status_> {
	public:
		/**
		  * @brief The constructor for this controller.
		  * @param name The name of this component.
		  * Every top-level controller will have this name parameter,
		  * just like current controllers.
		  */
		ATCSlipRunning(string name);
	
	private:
		/**
		  * @brief This is the main function for the top-level controller.
		  * The ATC class automatically handles startup and shutdown,
		  * if they are not disabled.
		  */
		void controller();


		/**
		  * @brief These are function within the top-level controller.
		  */
		void updateState();

		
		/**
		  * @brief These are function within the top-level controller.
		  */
		void hipController();

		
		/**
		  * @brief These are function within the top-level controller.
		  */
		void standingController();

		
		/**
		  * @brief These are function within the top-level controller.
		  */
		void shutdownController();

		
		void rightLegFlightFalling();
		void rightLegStance();
		void leftLegFlightRising();
		void leftLegFlightFalling();
		void leftLegStance();
		void rightLegFlightRising();
		double equilibriumGaitSolver(double vx, double va, double r);
		

		/**
		  * @brief These are sub controllers used by the top level controller.
		  */
  		ASCCommonToolkit ascCommonToolkit;
  		ASCInterpolation ascInterpolation;
		ASCSlipModel ascSlipModel;
		ASCLegForce ascLegForceLl;
		ASCLegForce ascLegForceRl;
		ASCHipBoomKinematics ascHipBoomKinematics;
		ASCPD ascPDLmA;
		ASCPD ascPDLmB;
		ASCPD ascPDRmA;
		ASCPD ascPDRmB;
		ASCPD ascPDLh;
		ASCPD ascPDRh;
		ASCRateLimit ascRateLimitLmA;
		ASCRateLimit ascRateLimitLmB;
		ASCRateLimit ascRateLimitRmA;
		ASCRateLimit ascRateLimitRmB;
		
		
		// Declare variables
		int controllerState, runningState;
		int stanceControlType;
		
		// For cubic interpolations
		double t, dt, t1, t2;
		double qLl1, qLl2, dqLl1, dqLl2, qRl1, qRl2, dqRl1, dqRl2;
		double rLl1, rLl2, drLl1, drLl2, rRl1, rRl2, drRl1, drl2;
		
		// SLIP model variables
		SlipState slipState;
		LegForce legForce;		
		
		// Hip variables
		double qLh, qRh;
		LeftRight toePosition;
		
		// Motor and leg variables
		double legRateLimit;
		double qLl, rLl, qRl, rRl;
		double dqLl, drLl, dqRl, drRl;
		double qLmA, qLmB, qRmA, qRmB;
		double dqLmA, dqLmB, dqRmA, dqRmB;
		
		// Leg cartesian lengths for ground triggers
		double xRl, zRl, xLl, zLl;
		
		// Equilibrium gait curve fit
		double b[55];
		double q, dq;
				
		double k, dk;
};

}
}

#endif // ATC_SLIP_RUNNING_HPP
