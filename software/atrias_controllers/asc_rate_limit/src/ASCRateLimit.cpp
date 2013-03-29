#include "asc_rate_limit/ASCRateLimit.hpp"

// Again, we need to put our code inside the appropriate namespaces.
namespace atrias {
namespace controller {

/* Breakdown of the next few lines:
 * We need to call our parent class's constructor,
 * then we can call the LogPort's constructor. The second parameter
 * is the name for this log port, which controls how it appears in the
 * bagfiles.
 */
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
	log_out.data.tgt     = tgt;
	log_out.data.posRate = posRate;
	log_out.data.negRate = negRate;

	// Our delta time
	double dt = ((double) CONTROLLER_LOOP_PERIOD_NS) / ((double) SECOND_IN_NANOSECONDS);

	// Compute the actual output command
	// I'm just placing this right in the log data for convenience.
	log_out.data.out = clamp(tgt, log_out.data.out + dt * negRate, log_out.data.out + dt * posRate);

	// Transmit the log data
	log_out.send();

	// Return our output command -- sending the log data does not change
	// this value.
	return log_out.data.out;
}

}
}

// vim: noexpandtab
