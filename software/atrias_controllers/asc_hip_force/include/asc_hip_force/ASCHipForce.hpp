#ifndef ASCHipForce_HPP
#define ASCHipForce_HPP

/**
 * @file ASCHipForce.hpp
 * @author Ryan Van Why
 * @brief This controls one hip on the ATRIAS biped.
 */

// The include for the controller class
#include <atrias_control_lib/AtriasController.hpp>

// Subcontrollers
#include "asc_pd/ASCPD.hpp"
#include "asc_toe_decode/ASCToeDecode.hpp"

// Necessary data type
#include <atrias_msgs/robot_state_leg.h>

// Namespaces we're using
using namespace std;

// Our namespaces
namespace atrias {
namespace controller {

// The subcontroller class itself
class ASCHipForce : public AtriasController {
	public:
		/**
		  * @brief The constructor for this subcontroller
		  * @param parent The instantiating, "parent" controller.
		  * @param name   The name for this controller (such as "pdLeftA")
		  */
		ASCHipForce(AtriasController *parent, string name);

		/**
		  * @brief The main function for this controller.
		  * @param leg The robot_state_leg structure for this leg
		  * @return A commanded output.
		  */
		double operator()(const atrias_msgs::robot_state_leg &leg);

		/**
		  * @brief This returns whether or not the toe is touching the ground.
		  * @return True if the toe's touching the ground, false otherwise.
		  * This must be called after the above operator() function.
		  */
		bool onGround();

		// Gains for flight phase (position control)
		double flightP;
		double flightD;

		// Gains for stance phase (strain control) Note: Very different from flight phase gains!
		double stanceP;
		double stanceD;

		// Gains for toe decode controller
		double toeFilterGain;
		double toeThreshold;

	private:

		// PD controller for flight
		ASCPD         flightPD;
		// PD controller for stance
		ASCPD         stancePD;
		// Toe decoding subcontroller
		ASCToeDecode toeDecode;
};

// End namespaces
}
}

#endif // ASCHipForce_HPP

// vim: noexpandtab
