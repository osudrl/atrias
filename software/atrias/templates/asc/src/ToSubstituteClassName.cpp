/*! \file ToSubstituteClassName.cpp
 *  \brief Orocos Component code for the ToSubstitutePackageName subcontroller.
 */

#include "ToSubstitutePackageName/ToSubstituteClassName.hpp"

// Again, we need to put our code inside the appropriate namespaces.
namespace atrias {
namespace controller {

/* Breakdown of the next few lines:
 * We need to call our parent class's constructor,
 * then we can call the LogPort's constructor.
 */
ToSubstituteClassName::ToSubstituteClassName(AtriasController *parent, string name) :
	AtriasController(parent, name),
	log_out(this)
{
}

double ToSubstituteClassName::operator()() {
	// Transmit the log data
	log_out.send();

	// Return our output command -- sending the log data does not change
	// this value.
	return 0;
}

}
}
