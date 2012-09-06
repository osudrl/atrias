#ifndef ROBOT_VARIANT_DEFS_H
#define ROBOT_VARIANT_DEFS_H

// This is for biped #1

#define NUM_MEDULLAS_ON_ROBOT    7

// These have been set
#define LEFT_LEG_A_CALIB_VAL     263651250LL
#define LEFT_LEG_B_CALIB_VAL     262942179LL
// These have not been set yet.
#define RIGHT_LEG_A_CALIB_VAL    352351997LL
#define RIGHT_LEG_B_CALIB_VAL    348884314LL

// These have not been set yet
#define LEFT_LEG_A_RAD_PER_CNT    9.8039216e-09
#define LEFT_LEG_B_RAD_PER_CNT    -9.8039216e-09
#define RIGHT_LEG_A_RAD_PER_CNT   9.8039216e-09
#define RIGHT_LEG_B_RAD_PER_CNT   9.8039216e-09

// These have been set.
#define LEFT_TRAN_A_CALIB_VAL    199387520LL
#define LEFT_TRAN_B_CALIB_VAL    197437370LL
// These have not been set yet.
#define RIGHT_TRAN_A_CALIB_VAL   284287500
#define RIGHT_TRAN_B_CALIB_VAL   285247647

// These have not been set yet
#define LEFT_TRAN_A_RAD_PER_CNT   9.8039216e-09
#define LEFT_TRAN_B_RAD_PER_CNT   -9.8039216e-09
#define RIGHT_TRAN_A_RAD_PER_CNT  9.8039216e-09
#define RIGHT_TRAN_B_RAD_PER_CNT  9.8039216e-09

#define LEFT_MOTOR_A_DIRECTION    -1.0
#define LEFT_MOTOR_B_DIRECTION     1.0
#define RIGHT_MOTOR_A_DIRECTION   -1.0
#define RIGHT_MOTOR_B_DIRECTION    1.0

/** @brief The pitch encoder's reading when the robot is vertical.
  * In encoder ticks.
  * Not set yet.
  */
#define BOOM_PITCH_VERTICAL_VALUE  128115

#endif // ROBOT_VARIANT_DEFS_H
