#ifndef ROBOT_VARIANT_DEFS_H
#define ROBOT_VARIANT_DEFS_H

// This is for biped #3

// Include the calibration values for each leg
#define INCLUDE_LEFT_LEG
#include "leg5_definitions.h"
#undef INCLUDE_LEFT_LEG

#define INCLUDE_RIGHT_LEG
#include "leg6_definitions.h"
#undef INCLUDE_RIGHT_LEG

/** @brief The pitch encoder's reading when the robot is vertical.
  * In encoder ticks.
  */
#define BOOM_PITCH_VERTICAL_VALUE  45741

/** @brief Boom height
  * Distance from the ground plane measured vertically to the boom-base Z-axis pivot.
  * Measured with a measuring tape. [m]
  */
#define BOOM_HEIGHT                1.0071125

/** @brief Boom length.
  * Distance from boom-base pivot Z-axis to intersection of robot torso XZ-center-plane and boom Y-axis centerline.
  * Measured from SolidWorks model. [m]
  */
#define BOOM_LENGTH                2.006

/** @brief Boom torso offset
  * The angular offset between the boom Y-axis centerline and the torso XZ-centerplane.
  * Measured from SolidWorks model. [rad]
  */
#define BOOM_TORSO_OFFSET          1.6968

/** @brief Torso length.
  * Distance from boom Y-axis intersection with robot body XZ-center-plane to hip pivot X-axis.
  * Measured from SolidWorks model. [m]
  */
#define TORSO_LENGTH               0.3434

/** @brief Hip length
  * Distance from the torso-hip X-axis pivot and the XZ-centerplane of the leg assembly.
  * Measured from SolidWorks model. [m]
  */
#define HIP_LENGTH                 0.18

/** @brief Proximal leg length
  * Length of the proximal leg segment.
  * Measured from robot. [m]
  */
#define PROXIMAL_LEG_LENGTH        0.5112

/** @brief Distal leg length
  * Length of the distal leg segment.
  * Measured from  robot. [m]
  */
#define DISTAL_LEG_LENGTH          0.508

/** @brief The meters of boom motion per encoder tick.
  * This is calculated from the boom's length, the number of encoder ticks per
  * encoder revolution, and the gear ratio between the boom and the encoder.
  */
#define BOOM_X_METERS_PER_TICK     -0.00000937522094511213193198

/** @brief The angle of the boom at the calibration location.
  */
#define BOOM_Z_CALIB_LOC           2.951490384953993

/** @brief The value of the boom encoder at the calibration location.
 *
 * This should be calibrated before BOOM_Z_CALIB_LOC!
  */
#define BOOM_Z_CALIB_VAL           17470

/** @brief The encoder value of the hip's absolute encoder at the calibration position.
  */
#define LEFT_HIP_CALIB_VAL         5737

/** @brief The encoder value of the right hip at calibration.
  */
#define RIGHT_HIP_CALIB_VAL        444

/** @brief The left hip's calibration position.
  */
#define LEFT_HIP_CALIB_POS         4.55007335

/** @brief The right hip's calibration position.
  */
#define RIGHT_HIP_CALIB_POS        4.89041256

/** @brief Maximum motor torque for scaling
  */
#define MTR_MAX_CURRENT            60.0
#define MTR_HIP_MAX_CURRENT        60.0

/** @brief Main motor torque limits. These are not used for scaling, just as limits.
  */
#define MAX_MTR_CURRENT_CMD        60.0
#define MIN_MTR_CURRENT_CMD        -60.0
#define MAX_HIP_MTR_CURRENT_CMD    60.0
#define MIN_HIP_MTR_CURRENT_CMD    -60.0

//! The threshold (above calibrated startup value) at which the toes detect contact
#define TOE_THRESH                  500

#endif // ROBOT_VARIANT_DEFS_H
