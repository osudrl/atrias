#ifndef ASC_COMMON_TOOLKIT_HPP
#define ASC_COMMON_TOOLKIT_HPP
 
/**
  * @file ASC_COMMON_TOOLKIT.hpp
  * @author Mikhail Jones
  * @brief This implements common functionality required by most controllers.
  */

// The include for the controller class
#include <atrias_control_lib/AtriasController.hpp>

// And for the logging helper class
#include <atrias_control_lib/LogPort.hpp>

// Our log data
#include "asc_common_toolkit/controller_log_data.h"

// Datatypes
#include <atrias_shared/controller_structs.h>
#include <atrias_shared/atrias_parameters.h>

// Namespaces we're using
using namespace std;
//using namespace atrias_msgs;

// Our namespaces
namespace atrias {
namespace controller {

// The subcontroller class itself
class ASCCommonToolkit : public AtriasController {
        public:
                /**
                  * @brief The constructor for this subcontroller
                  * @param parent The instantiating, "parent" controller.
                  * @param name The name for this controller.
                  */                  
                ASCCommonToolkit(AtriasController *parent, string name);

                /**
                  * @brief The leg stiffness function.
                  * @param r The current leg length.
                  * @param r0 The initial leg length.
                  * @return k The computed virtual leg stiffness.
                  */
				double legStiffness(double r, double r0);
				double k;
				
                /**
                  * @brief The polar motor position to leg position function.
                  * @param qmA The motor A angular position.
                  * @param qmB The motor B agular position.
                  * @return ql The computed leg angle.
                  * @return rl The computed leg length.
                  */
				std::tuple<double, double> polMotorPos2LegPos(double qmA, double qmB);
				double ql, rl;
				
                /**
                  * @brief The polar leg position to motor position function.
                  * @param ql The current leg angle.
                  * @param rl The initial leg length.
                  * @return qmA The computed motor A angular position
                  * @return qmB The computed motor B angular position.
                  */
				std::tuple<double, double> polLegPos2MotorPos(double ql, double rl);
				double qmA, qmB;
				
                /**
                  * @brief The polar leg velocity to motor velocity function.
                  * @param ql The current leg angle.
                  * @param dql The current leg angular velocity.
                  * @param drl The current leg length velocity.
                  * @return dqmA The computed motor A velocity.
                  * @return dqmB The computed motor B velocity.
                  */
				std::tuple<double, double> polLegVel2MotorVel(double ql, double dql, double drl);
				double dqmA, dqmB;
				
				/**
                  * @brief The radian to degree conversion function.
                  * @param rad The current angle in radians.
                  * @return deg The computed angle in degrees.
                  */
				double rad2deg(double rad);
				double deg;
				
				/**
                  * @brief The degree to radian conversion function.
                  * @param deg The current angle in degrees.
                  * @return rad The computed angle in radians.
                  */
				double deg2rad(double deg);
				double rad;

       
        private:
                /** 
                  * @brief This is our logging port.
                  * You may have as many of these as you'd like of various types.
                  */
                LogPort<asc_common_toolkit::controller_log_data> log_out;
};

// End namespaces
}
}

#endif // ASC_COMMON_TOOLKIT_HPP
