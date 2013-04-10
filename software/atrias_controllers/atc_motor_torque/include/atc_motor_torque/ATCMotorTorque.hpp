#ifndef ATCMotorTorque_HPP
#define ATCMotorTorque_HPP

/**
  * @file ATCMotorTorque.hpp
  * @author Ryan Van Why
  * @brief This implements motor torque control for every motor.
  */

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>
// No logging helper is needed -- one log port is automatically produced.

// The type transmitted from the GUI to the controller
#include "atc_motor_torque/controller_input.h"

// Namespaces we're using
using namespace std;

// Our namespaces
namespace atrias {
namespace controller {

class ATCMotorTorque : public ATC<Unused, atc_motor_torque::controller_input> {
	public:
		/**
		  * @brief The constructor for this controller.
		  * @param name The name of this component.
		  * Every top-level controller will have this name parameter,
		  * just like current controllers.
		  */
		ATCMotorTorque(string name);
	
	private:
		/**
		  * @brief This is the main function for the top-level controller.
		  */
		void controller();
};

}
}

#endif // ATCMotorTorque_HPP

// vim: noexpandtab
