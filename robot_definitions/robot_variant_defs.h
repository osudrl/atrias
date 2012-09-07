#ifndef ROBOT_VARIANT_DEFS_H
#define ROBOT_VARIANT_DEFS_H

// This is for biped #1

#define NUM_MEDULLAS_ON_ROBOT    7

// Include the calibration values for each leg
#define INCLUDE_LEFT_LEG
#include "leg2_definitions.h"
#undef INCLUDE_LEFT_LEG

#define INCLUDE_RIGHT_LEG
#include "leg1_definitions.h"
#undef INCLUDE_RIGHT_LEG

/** @brief The pitch encoder's reading when the robot is vertical.
  * In encoder ticks.
  * Not set yet.
  */
#define BOOM_PITCH_VERTICAL_VALUE  128115

#endif // ROBOT_VARIANT_DEFS_H
