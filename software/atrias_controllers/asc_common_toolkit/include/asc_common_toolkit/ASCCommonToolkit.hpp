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
                
                // Robot parameters
                double ks;


                /**
                  * @brief Computes the current axial leg stiffness.
                  * @param r The current leg length.
                  * @param r0 The initial leg length.
                  * @return k The computed virtual leg stiffness.
                  */
				double legStiffness(double r, double r0);
				double k;
				
				
                /**
                  * @brief Converts motor position to leg position.
                  * @param qmA The motor A angular position.
                  * @param qmB The motor B agular position.
                  * @return ql The computed leg angle.
                  * @return rl The computed leg length.
                  */
				std::tuple<double, double> motorPos2LegPos(double qmA, double qmB);
				double ql, rl;
				
				
                /**
                  * @brief Converts leg position to motor position.
                  * @param ql The current leg angle.
                  * @param rl The initial leg length.
                  * @return qmA The computed motor A angular position
                  * @return qmB The computed motor B angular position.
                  */
				std::tuple<double, double> legPos2MotorPos(double ql, double rl);
				double qmA, qmB;
				
				
				/**
                  * @brief Converts motor velocity to leg velocity.
                  * @param qmA The current motor A angle.
                  * @param qmB The current motor B angle.
                  * @param dqmA The current motor A velocity.
                  * @param dqmB The current motor B velocity.
                  * @return drl The computed leg length velocity.
                  * @return dql The computed leg angle velocity.
                  */
				std::tuple<double, double> motorVel2legVel(double qmA, double qmB, double dqmA, double dqmB);
				double drl, dql;
				
				
                /**
                  * @brief Converts leg velocity to motor velocity.
                  * @param ql The current leg angle.
                  * @param dql The current leg angular velocity.
                  * @param drl The current leg length velocity.
                  * @return dqmA The computed motor A velocity.
                  * @return dqmB The computed motor B velocity.
                  */
				std::tuple<double, double> legVel2MotorVel(double ql, double dql, double drl);
				double dqmA, dqmB;
				
				
				/**
                  * @brief Converts radians to degrees.
                  * @param rad The current angle in radians.
                  * @return deg The computed angle in degrees.
                  */
				double rad2deg(double rad);
				double deg;
				
				
				/**
                  * @brief Converts degrees to radians.
                  * @param deg The current angle in degrees.
                  * @return rad The computed angle in radians.
                  */
				double deg2rad(double deg);
				double rad;
				
				
				/**
                  * @brief Converts cartesian coordiantes to polar coordinates.
                  * @param x The x cartesian coordinate.
                  * @param z The z cartesian coordinate.
                  * @return q The q polar coordinate.
                  * @return r The r polar coordinate.
                  */
				std::tuple<double, double> cart2pol(double x, double z);
				double q, r;
				
				
				/**
                  * @brief Converts polar coordiantes to cartesian coordinates.
                  * @param q The q polar coordinate.
                  * @param r The r polar coordinate.
                  * @return x The x cartesian coordinate.
                  * @return z The z cartesian coordinate.
                  */
				std::tuple<double, double> pol2cart(double q, double r);
				double x, z;

       
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
