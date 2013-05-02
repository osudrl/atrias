#include "asc_rate_limit/ASCRateLimit.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

// Controller constructor
ASCRateLimit::ASCRateLimit(AtriasController *parent, string name) :
	AtriasController(parent, name),
	log_out(this, "log")
{
	// No init required for this controller
}


double ASCRateLimit::operator()(double tgt, double rate) {

	return (*this)(tgt, rate, -rate);
	
}


double ASCRateLimit::operator()(double tgt, double posRate, double negRate) {

	// Log our input
	log_out.data.tgt = tgt;
	log_out.data.posRate = posRate;
	log_out.data.negRate = negRate;

	// Compute the delta time
	double dt = ((double) CONTROLLER_LOOP_PERIOD_NS) / ((double) SECOND_IN_NANOSECONDS);

	// Compute the actual output command
	log_out.data.out = clamp(tgt, log_out.data.out + dt * negRate, log_out.data.out + dt * posRate);

	// Transmit the log data
	log_out.send();

	// Return our output command
	return log_out.data.out;
	
}


double ASCRateLimit::reset(double new_value) {

	return log_out.data.out = new_value;
	
};

}
}

// vim: noexpandtab
