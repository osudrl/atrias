#ifndef ATCSlipHopping_HPP
#define ATCSlipHopping_HPP

/**
 * @file ATC_SLIP_HOPPING.hpp
 * @author Mikhail S. Jones
 * @brief This hopping controller is based on a Spring Loaded Inverted 
 * Pendulum (SLIP) template model.
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
		
		/**
		  * @brief These are functions for the top-level controller.
		  */
		void updateController();
		void updateGui();
		void updateSlipConditions();
		void updateSlipForces();
		void hipController();
		void standingController();
		void slipForceStanceController(atrias_msgs::robot_state_leg*, atrias_msgs::controller_output_leg*, ASCPD*, ASCPD*, ASCLegForce*);
		void passiveStanceController(atrias_msgs::robot_state_leg*, atrias_msgs::controller_output_leg*, ASCPD*, ASCPD*);
		void virtualSpringStanceController(atrias_msgs::robot_state_leg*, atrias_msgs::controller_output_leg*, ASCLegForce*);
		void stanceEvents();
		void ballisticFlightLegController(atrias_msgs::robot_state_leg*, atrias_msgs::controller_output_leg*, ASCPD*, ASCPD*, ASCRateLimit*, ASCRateLimit*);
		void ballisticStanceLegController(atrias_msgs::robot_state_leg*, atrias_msgs::controller_output_leg*, ASCPD*, ASCPD*, ASCRateLimit*, ASCRateLimit*);
		void ballisticEvents();
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

		// Motor and leg variables
		double rSl, drSl, qSl, dqSl; // Stance leg states
		double rFl, drFl, qFl, dqFl; // Flight leg states
		double qSmA, dqSmA, qSmB, dqSmB; // Stance motor states
		double qFmA, dqFmA, qFmB, dqFmB; // Flight motor states
		LegForce forceS, forceF;
		
		// Motor and leg variables
		double rLl, drLl, qLl, dqLl; // Left leg states
		double rRl, drRl, qRl, dqRl; // Right leg states
		double qLmA, dqLmA, qLmB, dqLmB; // Left motor states
		double qRmA, dqRmA, qRmB, dqRmB; // Right motor states
		LegForce forceL, forceR;

		// State transistion events
		bool isLeftLegTO, isLeftLegTD, isRightLegTO, isRightLegTD;
		double forceThresholdTO, forceThresholdTD, positionThresholdTD;

		// Hip controller
		double qLh, qRh; // Hip angles
		LeftRight toePosition; // Desired toe positions measures from boom center axis
				
		// Controller options and parameters
		int controllerState, hoppingState; // State machine references
		double legRateLimit, hipRateLimit; // Velocity limit for motors (used in standing controller only)
		int stanceControlType, hoppingType, forceControlType, springType;
		
		// Desired hop height for terrain following SLIP force controller
		double hopHeight;
		
		// SLIP model state structure
		SlipState slipState;
		
		// Simulated virtual spring between robot
		double k, dk;

};

}
}

#endif // ATCSlipHopping_HPP
