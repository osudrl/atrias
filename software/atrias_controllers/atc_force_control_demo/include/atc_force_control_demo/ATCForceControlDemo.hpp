#ifndef ATC_FORCE_CONTROL_DEMO_HPP
#define ATC_FORCE_CONTROL_DEMO_HPP

/**
  * @file ATC_FORC_CONTROL_DEMO.hpp
  * @author Mikhail Jones
  * @brief This implements a demo force controller.
  */

// Top-level controllers are components, so we need to include this.
#include <rtt/Component.hpp>

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>
// No logging helper is needed -- one log port is automatically produced.

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
using namespace atc_force_control_demo;

// Our namespaces
namespace atrias {
namespace controller {

class ATCForceControlDemo : public ATC<atc_force_control_demo::controller_log_data, controller_input, controller_status> {
	public:
		/**
		  * @brief The constructor for this controller.
		  * @param name The name of this component.
		  */
		ATCForceControlDemo(string name);
	
	private:
		/**
		  * @brief This is the main function for the top-level controller.
		  * The ATC class automatically handles startup and shutdown,
		  * if they are not disabled.
		  */
		void controller();

		/**
		  * @brief This gets GUI values and handles some high level logic.
		  */		  
		void updateState();
		
		/**
		  * @brief Constant toe position hip controller.
		  */	
		void hipController();
		
		/**
		  * @brief Runs through a predetermined set of positions.
		  */	
		std::tuple<double, double, double, double> automatedPositionTest(double t);
		
		/**
		  * @brief Runs through a predetermined set of forces.
		  */	
		LegForce automatedForceTest(double t);
		
		/**
		  * @brief These are sub controllers used by the top level controller.
		  */
		ASCCommonToolkit ascCommonToolkit;
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
		
		// Time counters
		double tL, tR;
		
		// Leg and motor positions
		double qLmA, qLmB, qRmA, qRmB, dqLmA, dqLmB, dqRmA, dqRmB;
		
		// Leg motor rate limit
		double legRateLimit;
		
		// Leg forces
		LegForce fL, fR, fTemp;
		
		// Controller states
		int lLegControllerState, rLegControllerState;
		
		// Hip angles
		double qLh, qRh;
		
		// Toe positions
		LeftRight toePosition;
		
		// Motor angles and velocities
		double qmA, qmB, dqmA, dqmB;
		
		// Leg force structure
		LegForce legForce;
		
		// Cycle duration for automated tests
		double duration;
		
		// Sine wave sweep parameters
		double omega1, omega2, a, b, ql, rl, dql, drl;

};

}
}

#endif // ATC_FORCE_CONTROL_DEMO_HPP
