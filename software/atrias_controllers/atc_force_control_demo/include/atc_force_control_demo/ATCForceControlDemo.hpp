#ifndef ATC_FORCE_CONTROL_DEMO_HPP
#define ATC_FORCE_CONTROL_DEMO_HPP

/**
  * @file ATC_FORC_CONTROL_DEMO.hpp
  * @author Mikhail Jones
  * @brief This implements a demo force controller.
  */

// Top-level controllers are components, so we need to include this.
#include <atrias_control_lib/ATC.hpp>
// Our logging data type.
#include "atc_force_control_demo/controller_log_data.h"
// The type transmitted from the GUI to the controller
#include "atc_force_control_demo/controller_input.h"
// The type transmitted from the controller to the GUI
#include "atc_force_control_demo/controller_status.h"

// Our subcontroller types
#include <asc_common_toolkit/ASCCommonToolkit.hpp>
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

// Our namespaces
namespace atrias {
namespace controller {

class ATCForceControlDemo : public ATC<
	atc_force_control_demo::controller_log_data_,
	atc_force_control_demo::controller_input_,
	atc_force_control_demo::controller_status_>
{
	public:
		/**
		  * @brief The constructor for this controller.
		  * @param name The name of this component.
		  */
		ATCForceControlDemo(string name);
	
	private:
		/**
		  * @brief This is the main function for the top-level controller.
		  */
		void controller();

		/**
		  * @brief These are functions for the top-level controller.
		  */
		void updateState();
		void hipController();
		std::tuple<double, double> sinewave(double, double, double, double);
		std::tuple<double, double> stairStep(double, double, double, double, double);
		std::tuple<double, double> sinewaveSweep(double, double, double, double, double, double);
			
		/**
		  * @brief These are sub controllers used by the top level controller.
		  */
		ASCCommonToolkit ascCommonToolkit;
		ASCHipBoomKinematics ascHipBoomKinematics;
		ASCLegForce ascLegForceL, ascLegForceR;
		ASCPD ascPDLmA, ascPDLmB, ascPDRmA, ascPDRmB, ascPDLh, ascPDRh;
		ASCRateLimit ascRateLimitLmA, ascRateLimitLmB, ascRateLimitRmA, ascRateLimitRmB, ascRateLimitLh, ascRateLimitRh;

		// Variables
		int lLegControllerState, rLegControllerState; // State machines
		double qLh, qRh; // Hip angles
		LeftRight toePosition; // Desired toe positions measures from boom center axis
		
		// Motor positions and velocities
		double qmA, qmB, dqmA, dqmB;
		
		// Leg positions and velocities
		double ql, rl, dql, drl;
		
		// 
		double t, y, dy;
		
		// Time counters
		double tL, tR;
		
		// Leg forces
		LegForce legForce;		

		// Sine wave sweep parameters
		double omega1, omega2, a, b;
		
		// Misc margins, ratelimiters and other kludge values
		double legRateLimit, hipRateLimit;

};

}
}

#endif // ATC_FORCE_CONTROL_DEMO_HPP

// vim: noexpandtab
