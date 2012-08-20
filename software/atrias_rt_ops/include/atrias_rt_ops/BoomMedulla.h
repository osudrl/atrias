#ifndef BOOMMEDULLA_H
#define BOOMMEDULLA_H

/** @file
  * @brief Contains all code specific to the boom medulla.
  */

class BoomMedulla;

// SOEM
extern "C" {
#include <ethercattype.h>
#include <ethercatmain.h>
}

#include "robot_invariant_defs.h"
#include "robot_variant_defs.h"
#include "atrias_rt_ops/RTOps.h"
#include "atrias_rt_ops/Medulla.h"

class BoomMedulla : public Medulla {
	RTOps* rtOps;
	
	// Stuff sent to the Medulla
	
	// Stuff received from the Medulla
	
};

#endif // BOOMMEDULLA_H
