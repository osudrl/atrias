#ifndef ROBOCOMM_H
#define ROBOCOMM_H

/** @file Contains a class representing the entire robot's IO data. */

#include "TestMedulla.h"
#include <stddef.h>

//! @brief Contains pointers to data for each of the robot's Medullas
class RobotComm {
	public:
		TestMedulla* test1;
		RobotComm();
};

#endif // ROBOCOM_H
