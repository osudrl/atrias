#ifndef ATCJointPosition_HPP
#define ATCJointPosition_HPP

/**
  * @file ATCJointPosition.hpp
  * @author Ryan Van Why
  * @brief This positions the robot into a desired state.
  */

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>

// The type transmitted from the GUI to the controller
#include "atc_joint_position/controller_input.h"
// Our log data type
#include "atc_joint_position/controller_log_data.h"

// Our subcontroller types
#include <asc_common_toolkit/ASCCommonToolkit.hpp>
#include <asc_hip_boom_kinematics/ASCHipBoomKinematics.hpp>
#include <asc_pd/ASCPD.hpp>
#include <asc_rate_limit/ASCRateLimit.hpp>

// Namespaces we're using
using namespace std;
using namespace atrias_msgs;

// Our namespaces
namespace atrias {
namespace controller {

class ATCJointPosition : public ATC<atc_joint_position::controller_log_data_, atc_joint_position::controller_input_> {
	public:
		/**
		  * @brief The constructor for this controller.
		  * @param name The name of this component.
		  * Every top-level controller will have this name parameter,
		  * just like current controllers.
		  */
		ATCJointPosition(string name);
	
	private:
		/**
		  * @brief This is the main function for the top-level controller.
		  */
		void controller();

		// Handles one leg of the controller
		AB runLeg(robot_state_leg &leg, double P, double D, double ang, double len,
			ASCRateLimit &rateLimA, ASCRateLimit &rateLimB, ASCPD &pdA, ASCPD &pdB);

		// PD controllers for each motor
		ASCPD pdLA; // Left  A
		ASCPD pdLB; // Left  B
		ASCPD pdLH; // Left  Hip
		ASCPD pdRA; // Right A
		ASCPD pdRB; // Right B
		ASCPD pdRH; // Right Hip

		// Rate limiters for each motor
		ASCRateLimit rateLimLA; // Left  A
		ASCRateLimit rateLimLB; // Left  B
		ASCRateLimit rateLimLH; // Left  Hip
		ASCRateLimit rateLimRA; // Right A
		ASCRateLimit rateLimRB; // Right B
		ASCRateLimit rateLimRH; // Right Hip

		// For converting leg geometries into motor positions
		ASCCommonToolkit commonToolkit;

		// For hip control
		ASCHipBoomKinematics hipKine;
};

}
}

#endif // ATCJointPosition_HPP

// vim: noexpandtab
