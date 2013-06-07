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
#include <asc_hip_boom_kinematics/ASCHipBoomKinematics.hpp>
#include <asc_slip_model/ASCSlipModel.hpp>
#include <asc_interpolation/ASCInterpolation.hpp>
#include <asc_leg_force/ASCLegForce.hpp>
#include <asc_pd/ASCPD.hpp>
#include <asc_rate_limit/ASCRateLimit.hpp>

// Datatypes
#include <robot_invariant_defs.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_shared/controller_structs.h>
#include <atrias_shared/atrias_parameters.h>

// Namespaces we're using
using namespace std;

// Our namespaces
namespace atrias {
namespace controller {

class ATCSlipHopping : public ATC<
	atc_slip_hopping::controller_log_data_,
	atc_slip_hopping::controller_input_,
	atc_slip_hopping::controller_status_> 
{
	public:
		/**
		  * @brief The constructor for this controller.
		  * @param name The name of this component.
		  */
		ATCSlipHopping(string name);

	private:
		/**
		  * @brief This is the main function for the top-level controller.
		  * The ATC class automatically handles startup and shutdown,
		  * if they are not disabled.
		  */
		void controller();
		double legRateLimit, hipRateLimit; // Velocity limit for motors (used in standing controller only)

		/**
		  * @brief These are functions for the top-level controller.
		  */
		void updateState();
		int controllerState, hoppingState; // State machine references
		
		void hipController();
		double qLh, qRh; // Hip angles
		LeftRight toePosition; // Desired toe positions measures from boom center axis
		
		void standingController();
		
		void slipForceStancePhaseController();
		void passiveStancePhaseController();
		void virtualSpringStancePhaseController();
		void flightPhaseController();
		void shutdownController();
		
		/**
		  * @brief These are sub controllers used by the top level controller.
		  */
		ASCCommonToolkit ascCommonToolkit;
		ASCHipBoomKinematics ascHipBoomKinematics;
		ASCInterpolation ascInterpolation;
		ASCSlipModel ascSlipModel;
		ASCLegForce ascLegForceL, ascLegForceR;
		ASCPD ascPDLmA, ascPDLmB, ascPDRmA, ascPDRmB, ascPDLh, ascPDRh;
		ASCRateLimit ascRateLimitLmA, ascRateLimitLmB, ascRateLimitRmA, ascRateLimitRmB, ascRateLimitLh, ascRateLimitRh;
		
		
		
		// Controller options and parameters
		int stanceControlType, hoppingType, forceControlType, springType;
		
		// Defines which legs are used in stance
		bool isLeftStance, isRightStance;
			
		// Leg angles and lengths	
		double qLl, rLl, qRl, rRl;
		
		// Leg velocities
		double dqLl, drLl, dqRl, drRl;
		
		// Motor angles and velocities
		double qLmA, qLmB, qRmA, qRmB;
		
		
		
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
