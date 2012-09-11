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

/** @brief The length of the boom. For Z calculations.
  * This is measured from the center of rotation to the center of the hip's
  * rotation on the body.
  * Measured via measuring tape.
  */
#define BOOM_LENGTH                2.006

/** @brief The vertical distance between the boom arm's centerline and the robot
  * coordinate system's center (Z = 0) when the boom is level.
  * Measured off the SolidWorks model.
  */
#define BOOM_ROBOT_VERTICAL_OFFSET 0.3434

/** @brief The height of the center of rotation for the boom arm, relative to
  * robot's "ground".
  * Measured with a measuring tape.
  */
#define BOOM_HEIGHT                0.915

/** @brief The meters of boom motion per encoder tick.
  * This is calculated from the boom's length, the number of encoder ticks per
  * encoder revolution, and the gear ratio between the boom and the encoder.
  */
#define BOOM_X_METERS_PER_TICK     -0.00000937522094511213193198

/** @brief The angle of the boom at the calibration location.
  */
#define BOOM_Z_CALIB_LOC            3.141592653589793238462643383279502884197169

/** @brief The value of the boom encoder at the calibration location.
  */
#define BOOM_Z_CALIB_VAL            1

/** @brief The encoder value of the hip's absolute encoder at the calibration position.
  */
#define LEFT_HIP_CALIB_VAL          1

/** @brief The encoder value of the right hip at calibration.
  */
#define RIGHT_HIP_CALIB_VAL         1

/** @brief The left hip's calibration position.
  */
#define LEFT_HIP_CALIB_POS           4.0

/** @brief The right hip's calibration position.
  */
#define RIGHT_HIP_CALIB_POS          4.0

#endif // ROBOT_VARIANT_DEFS_H
