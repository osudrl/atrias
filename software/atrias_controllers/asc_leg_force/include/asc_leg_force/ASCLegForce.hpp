#ifndef ASC_LEG_FORCE_HPP
#define ASC_LEG_FORCE_HPP
 
/**
  * @file ASC_LEG_FORCE.hpp
  * @author Mikhail Jones
  * @brief This implements leg force functions.
  */

// The include for the controller class
#include <atrias_control_lib/AtriasController.hpp>

// And for the logging helper class
#include <atrias_control_lib/LogPort.hpp>

// Our log data
#include "asc_leg_force/controller_log_data.h"

// Datatypes
#include <atrias_shared/controller_structs.h>
#include <atrias_shared/atrias_parameters.h>
#include <atrias_msgs/robot_state.h>

// Namespaces we're using
using namespace std;
using namespace atrias_msgs;

// Our namespaces
namespace atrias {
namespace controller {

// The subcontroller class itself
class ASCLegForce : public AtriasController {
    public:
		/**
		  * @brief The constructor for this subcontroller
		  * @param parent The instantiating, "parent" controller.
		  * @param name The name for this controller.
		  */                  
		ASCLegForce(AtriasController *parent, string name);
		
		// Angular positions
		double qlA, qlB, qmA, qmB, qb;

		// Angular velocities
		double dqlA, dqlB, dqmA, dqmB, dqb;

		// Spring torques
		double tausA, tausB, dtausA, dtausB;
		
		/**
		  * @brief The leg force control function.
		  * @param legForce
		  * @param leg
		  * @param position
		  * @return motorCurrent The computed motor current.
		  */
		std::tuple<double, double> control(LegForce legForce, atrias_msgs::robot_state_leg leg, atrias_msgs::robot_state_location position);

		// Gains
		double kp, ki, kd;
		
		// Forces
		double fx, fz, dfx, dfz;

		// Motor current
		double curA, curB;
	
		/**
		  * @brief The leg force function.
		  * @param leg
		  * @param position
		  * @return legForce The computed leg forces.
		  */
		LegForce compute(atrias_msgs::robot_state_leg leg, atrias_msgs::robot_state_location position);		
		
		// Leg forces
		LegForce legForce;

    private:
		/** 
		  * @brief This is our logging port.
		  * You may have as many of these as you'd like of various types.
		  */
		LogPort<asc_leg_force::controller_log_data> log_out;
};

// End namespaces
}
}

#endif // ASC_LEG_FORCE_HPP
