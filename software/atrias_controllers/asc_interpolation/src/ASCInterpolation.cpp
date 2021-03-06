#include "asc_interpolation/ASCInterpolation.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

ASCInterpolation::ASCInterpolation(AtriasController *parent, string name) :
	AtriasController(parent, name),
	log_out(this)
{
	// Nothing to see here
}

std::tuple<double, double> ASCInterpolation::linear(double x1, double x2, double y1, double y2, double x, double dx) {

	// Limit range since curve fit is only valid within range
	 x = clamp(x, x1, x2);
	
	// Interpolate
	y = (x - x1)*(y2 - y1)/(x2 - x1) + y1;
	dy = dx*(y1 - y2)/(x1 - x2);

	// Set the log data
	log_out.data.linear_y = y;
	log_out.data.linear_dy = dy;

	// Transmit the log data
	log_out.send();

	// Return our output command
	return std::make_tuple(y, dy);
	
}


double ASCInterpolation::bilinear(double x1, double x2, double y1, double y2, double z11, double z21, double z12, double z22, double x, double y) {

	// Limit range since curve fit is only valid within range
	 x = clamp(x, x1, x2);
	 y = clamp(y, y1, y2);
	
	// Interpolate
	z = (1.0/(x2 - x1)*(y2 - y1))*(z11*(x2 - x)*(y2 - y) + z21*(x - x1)*(y2 - y) + z12*(x2 - x)*(y - y1) + z22*(x - x1)*(y - y1));

	// Set the log data
	log_out.data.bilinear_z = z;

	// Transmit the log data
	log_out.send();

	// Return our output command
	return z;
	
}

std::tuple<double, double> ASCInterpolation::cosine(double x1, double x2, double y1, double y2, double x, double dx) {

	// Limit range since curve fit is only valid within range
	 x = clamp(x, x1, x2);
	
	// Interpolate
	s = (1.0 - cos((x - x1)/(x2 - x1)*PI))/2.0;
	y = y1*(1.0 - s) + y2*s;
	ds = -(PI*sin((PI*(x1 - x))/(x1 - x2))*dx)/(2.0*(x1 - x2));
	dy = -y1*ds + y2*ds;

	// Set the log data
	log_out.data.cosine_y = y;
	log_out.data.cosine_dy = dy;

	// Transmit the log data
	log_out.send();

	// Return our output command
	return std::make_tuple(y, dy);
	
}


std::tuple<double, double> ASCInterpolation::cubic(double x1, double x2, double y1, double y2, double dy1, double dy2, double x, double dx) {

	// Limit range since curve fit is only valid within range
	 x = clamp(x, x1, x2);

	// Interpolate
	a0 = 2.0*(y1 - y2) + (dy1 + dy2)*(x2 - x1);
	a1 = y2 - y1 - dy1*(x2 - x1) - a0;
	a2 = dy1*(x2 - x1);
	a3 = y1;
	s = (x - x1)/(x2 - x1);
	y = a0*s*s*s + a1*s*s + a2*s + a3;
  dy = dx*(-3.0*a0*pow(x - x1, 2.0)/pow(x1 - x2, 3.0) + 2.0*a1*(x - x1)/pow(x1 - x2, 2.0) - a2/(x1 - x2));
	
	// Set the log data
	log_out.data.cubic_y = y;
	log_out.data.cubic_dy = dy;

	// Transmit the log data
	log_out.send();

	// Return our output command
	return std::make_tuple(y, dy);
	
}

}
}
