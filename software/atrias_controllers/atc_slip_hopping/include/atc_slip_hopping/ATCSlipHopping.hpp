#ifndef ATC_SLIP_HOPPING_HPP
#define ATC_SLIP_HOPPING_HPP

/**
  * @file ATC_SLIP_HOPPING.hpp
  * @author Mikhail Jones
  * @brief This implements a SLIP based template controller.
  */

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>
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
#include <asc_pd/ASCPD.hpp>
#include <asc_rate_limit/ASCRateLimit.hpp>

// Datatypes
#include <robot_invariant_defs.h>
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
class ATCSlipHopping : public ATC<atc_slip_hopping::controller_log_data_, controller_input_, controller_status_> {
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

		/**
		  * @brief Gets values from GUI and updates all relavent states.
		  */
		void updateState();

		/**
		  * @brief A kinematically driven hip controller to limit knee forces.
		  */
		void hipController();
		
		/**
		  * @brief A simple two leg standing controller that uses a position 
		  * controller to hold a constant virtual motor leg length.
		  */
		void standingController();
		
		/**
		  * @brief A SLIP based force tracking stance phase controller.
		  */
		void slipForceStancePhaseController();
		
		/**
		  * @brief A simple stance phase controller allowing only leg length 
		  * forces with zero leg angle torques. Uses a position controller to 
		  * keep virtual motor leg length constant while minimizing spring 
		  * about the hip.
		  */
		void passiveStancePhaseController();

		/**
		  * @brief A simple stance phase controller simulating a virtual spring
		  * between the hip and toe. Uses a force controller to then track these 
		  * forces that are based on leg deflection.
		  */
		void virtualSpringStancePhaseController();
		
		/**
		  * @brief A simple constant leg position flight phase controller.
		  */
		void flightPhaseController();
		
		/**
		  * @brief Applies virtual dampers to all motors.
		  */
		void shutdownController();
		
		/**
		  * @brief These are sub controllers used by the top level controller.
		  */
		ASCCommonToolkit ascCommonToolkit;
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
		
		// State machine references
		int controllerState, hoppingState;
		
		// Controller options and parameters
		int stanceControlType, hoppingType, forceControlType, springType;
		
		// Defines which legs are used in stance
		bool isLeftStance, isRightStance;
		
		// Hip angles
		double qLh, qRh;
		
		// Desired toe positions for hip boom kinematic controller
		LeftRight toePosition;
		
		// Leg angles and lengths	
		double qLl, rLl, qRl, rRl;
		
		// Leg velocities
		double dqLl, drLl, dqRl, drRl;
		
		// Motor angles and velocities
		double qLmA, qLmB, qRmA, qRmB;
		
		// Velocity limit for motors (used in standing controller only)
		double legRateLimit;
		
		// Desired hop height for terrain following SLIP force controller
		double h;
		
		// SLIP model state structure
		SlipState slipState;
		
		// Leg force structures
		LegForce legForce;
		
		// Simulated virtual spring between robot
		double k, dk;
		double qF, rF;

};

}
}

#endif // ATC_SLIP_HOPPING_HPP

// vim: noexpandtab
