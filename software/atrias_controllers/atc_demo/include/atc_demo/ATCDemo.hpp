#ifndef ATCDemo_HPP
#define ATCDemo_HPP

/**
 * @file ATCDemo.hpp
 * @author Mikhail Jones
 * @brief A controller to demonstrate range of motion, speed, precision.
 */

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>

// Our logging data type.
#include "atc_demo/controller_log_data.h"
// The type transmitted from the GUI to the controller
#include "atc_demo/controller_input.h"
// The type transmitted from the controller to the GUI
#include "atc_demo/controller_status.h"

// Include subcontrollers here

// Namespaces we're using
using namespace std;

// Our namespaces
namespace atrias {
namespace controller {

class ATCDemo : public ATC<
	atc_demo::controller_log_data_,
	atc_demo::controller_input_,
	atc_demo::controller_status_>
{
	public:
		/** 
		 * @brief The constructor for this controller.
		 * @param name The name of this component.
		 * Every top-level controller will have this name parameter,
		 * just like current controllers.
		 */
		ATCDemo(string name);

	private:
		/** 
		 * @brief This is the main function for the top-level controller.
		 */
		void controller();

		/**
		 * @brief These are sub controllers used by the top level controller.
		 */
		ASCCommonToolkit ascCommonToolkit;
		//ASCHipBoomKinematics ascHipBoomKinematics;
		//ASCInterpolation ascInterpolation;
		//ASCLegForce ascLegForceL, ascLegForceR;
		ASCPD ascPDLmA, ascPDLmB, ascPDRmA, ascPDRmB, ascPDLh, ascPDRh;
		ASCRateLimit ascRateLimitLmA, ascRateLimitLmB, ascRateLimitRmA, ascRateLimitRmB, ascRateLimitLh, ascRateLimitRh;

		/**
		 * @brief These are all the variables used by the top level controller.
		 */
 
		// Motor and leg variables
		double qLh, dqLh, qRh, dqRh; // Hip motor states
		double rLl, drLl, qLl, dqLl; // Left leg states
		double rRl, drRl, qRl, dqRl; // Right leg states
		double qmLA, dqmLA, qmLB, dqmLB; // Left leg motor states
		double qmRA, dqmRA, qmRB, dqmRB; // Right leg motor states

		// Misc margins, ratelimiters and other kludge values
	      double legRateLimit, hipRateLimit; // Rate limits
		double ql, dql, rl, drl; // Temp leg states

};

}
}

#endif // ATCDemo_HPP
