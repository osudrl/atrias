#ifndef ASC_HIP_BOOM_KINEMATICS_HPP
#define ASC_HIP_BOOM_KINEMATICS_HPP
 
/**
  * @file ASC_HIP_BOOM_KINEMATICS.hpp
  * @author Mikhail Jones
  * @brief This implements hip/boom kinematic relationships.
  */

// The include for the controller class
#include <atrias_control_lib/AtriasController.hpp>

// And for the logging helper class
#include <atrias_control_lib/LogPort.hpp>

// Our log data
#include "asc_hip_boom_kinematics/controller_log_data.h"

// Datatypes
#include <atrias_shared/controller_structs.h>
#include <atrias_shared/atrias_parameters.h>
#include <atrias_msgs/robot_state.h>

// Cpp
#include <complex.h>

// Namespaces we're using
using namespace std;
using namespace atrias_msgs;

// Our namespaces
namespace atrias {
namespace controller {

// The subcontroller class itself
class ASCHipBoomKinematics : public AtriasController {
        public:
		        /**
		          * @brief The constructor for this subcontroller
		          * @param parent The instantiating, "parent" controller.
		          * @param name The name for this controller.
		          */                  
		        ASCHipBoomKinematics(AtriasController *parent, string name);
	
				// Boom parameters
				double lBoom, lBody, lHip, qBodyOffset;

		        /**
		          * @brief The inverse kinematics function.
		          * @param toePosition
		          * @param lLeg
		          * @param rLeg
		          * @param position
		          * @return hipangle The computed hip angles.
		          */
				std::tuple<double, double> iKine(LeftRight toePosition, atrias_msgs::robot_state_leg lLeg, atrias_msgs::robot_state_leg rLeg, atrias_msgs::robot_state_location position);

				// Imaginary number
			   	complex<double> i;
			   	
			   	// Hip angles
				complex<double> complexHipAngleLeft, complexHipAngleRight;
	
				// Desired position
				double lLeftLeg, lRightLeg, qLeftLeg, qRightLeg;
	
				// Hip angles
				LeftRight hipAngle;
			
	   
        private:
                /** 
                  * @brief This is our logging port.
                  * You may have as many of these as you'd like of various types.
                  */
                LogPort<asc_hip_boom_kinematics::controller_log_data> log_out;
};

// End namespaces
}
}

#endif // ASC_HIP_BOOM_KINEMATICS_HPP
