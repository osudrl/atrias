/*! \file controller_component.cpp
 *  \author Andrew Peekema
 *  \brief Orocos Component code for the asc_component subcontroller.
 */

#include "asc_pd/ASCPD.hpp"

// Again, we need to put our code inside the appropriate namespaces.
namespace atrias {
namespace controller {

/* Breakdown of the next few lines:
 * We need to call our parent class's constructor,
 * then we can call the LogPort's constructor.
 */
ASCPD::ASCPD(AtriasController *parent, string name) :
	AtriasController(parent, name),
	log_out(this)
{
}

double ASCPD::operator()(double desPos, double curPos, double desVel, double curVel) {
	// Log our input data
	log_out.data.exampleData = 2.718;

	// Compute the actual output command
	// I'm just placing this right in the log data for convenience.
	log_out.data.output = 2.0 * M_PI;

	// Transmit the log data
	log_out.send();

	// Return our output command -- sending the log data does not change
	// this value.
	return log_out.data.output;
}

}
}

// vim: noexpandtab
