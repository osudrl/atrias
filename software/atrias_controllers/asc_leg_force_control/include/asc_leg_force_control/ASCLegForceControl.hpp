#ifndef ASC_LEG_FORCE_CONTROL_HPP
#define ASC_LEG_FORCE_CONTROL_HPP
 
/**
  * @file ASC_LEG_FORCE_CONTROL.hpp
  * @author Mikhail Jones
  * @brief This implements a leg force controller (PD).
  */

// The include for the controller class
#include <atrias_control_lib/AtriasController.hpp>

// And for the logging helper class
#include <atrias_control_lib/LogPort.hpp>

// Our log data
#include "asc_leg_force_control/controller_log_data.h"

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
class ASCLegForceControl : public AtriasController {
        public:
                /**
                  * @brief The constructor for this subcontroller
                  * @param parent The instantiating, "parent" controller.
                  * @param name The name for this controller.
                  */                  
                ASCLegForceControl(AtriasController *parent, string name);

                /**
                  * @brief The leg force to motor current function.
                  * @param
                  * @param
                  * @param
                  * @return motorCurrent The computed motor current.
                  */
				std::tuple<double, double> legForceToMotorCurrent(LegForce legForce, Gain gain, atrias_msgs::robot_state_leg leg, atrias_msgs::robot_state_location position);

				// Robot parameters
				double l1, l2;
		
				// Forces
				double fx, fz, dfx, dfz;
		
				// Angular positions
				double qlA, qlB, qmA, qmB, qb;
		
				// Angular velocities
				double dqlA, dqlB, dqmA, dqmB, dqb;
		
				// Gains
				double kp, kd, ks, kg, kt;
		
				// Torques
				double tausA, tausB, dtausA, dtausB;

				// Motor current
				AB motorCurrent;
				
       
        private:
                /** 
                  * @brief This is our logging port.
                  * You may have as many of these as you'd like of various types.
                  */
                LogPort<asc_leg_force_control::controller_log_data> log_out;
};

// End namespaces
}
}

#endif // ASC_LEG_FORCE_CONTROL_HPP
