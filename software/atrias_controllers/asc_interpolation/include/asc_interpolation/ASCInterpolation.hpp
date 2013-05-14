#ifndef __ASCInterpolation_HPP__
#define __ASCInterpolation_HPP__

/**
  * @file ASC_INTERPOLATION.hpp
  * @author Mikhail Jones
  * @brief This implements a few basic interpolation functions.
  */

// The include for the controller class
#include <atrias_control_lib/AtriasController.hpp>

// And for the logging helper class
#include <atrias_control_lib/LogPort.hpp>

// Our log data
#include "asc_interpolation/controller_log_data.h"

// Datatypes
#include <atrias_shared/atrias_parameters.h>

// Namespaces we're using
using namespace std;

// Our namespaces
namespace atrias {
namespace controller {

class ASCInterpolation : public AtriasController {
	public:
		/** 
		* @brief The constructor for this subcontroller
		* @param parent The instantiating, "parent" controller.
		* @param name   The name for this controller (such as "pdLeftA")
		*/
		ASCInterpolation(AtriasController *parent, string name);


		/** 
		* @brief Linear interpolation function.
		* @param x1, x2 The coordinates of the two points.
		* @param y1, y2 The values of the two points.
		* @param x The point to be interpolated.
		* @return y The interpolated value.
		*/
		std::tuple<double, double> linear(double x1, double x2, double y1, double y2, double x);
		
		
		/** 
		* @brief Bilinear interpolation function.
		* @param x1, x2, y1, y2 The coordinates of the four points.
		* @param z11, z12, z21, z22 The values of the four points.
		* @param x, y The point to be interpolated.
		* @return f The interpolated value.
		*/
		double bilinear(double x1, double x2, double y1, double y2, double z11, double z21, double z12, double z22, double x, double y);
		
		
		/** 
		* @brief Cosine interpolation function.
		* @param x1, x2 The coordinates of the two points.
		* @param y1, y2 The values of the two points.
		* @param x The point to be interpolated.
		* @return y The interpolated value.
		*/
		std::tuple<double, double> cosine(double x1, double x2, double y1, double y2, double x);
		
		
		/** 
		* @brief Cubic interpolation function.
		* @param x1, x2 The coordinates of the two points.
		* @param y1, y2 The values of the two points.
		* @param dy1, dy2 The slope of the two points.
		* @param x The point to be interpolated.
		* @return y The interpolated value.
		*/
		std::tuple<double, double> cubic(double x1, double x2, double y1, double y2, double dy1, double dy2, double x);


		// Declare variables
		double z, y, dy, s, ds, a0, a1, a2, a3;

	private:
		/** 
		* @brief This is our logging port.
		* You may have as many of these as you'd like of various types.
		*/
		LogPort<asc_interpolation::controller_log_data_> log_out;
};

}
}

#endif // __ASC_INTERPOLATION_HPP__
