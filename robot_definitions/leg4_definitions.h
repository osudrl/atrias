#ifndef LEG4_DEFINITIONS
#define LEG4_DEFINITIONS

/*------------------------------------------*/
/*------ Calibration Values for Leg 4 ------*/
/*------------------------------------------*/
#define LEG4_LEG_A_CALIB_VAL       265239580LL
#define LEG4_LEG_B_CALIB_VAL       174863762LL

#define LEG4_LEG_A_RAD_PER_CNT   9.8039216e-09
#define LEG4_LEG_B_RAD_PER_CNT   9.8039216e-09

#define LEG4_TRAN_A_CALIB_VAL      141352160LL
#define LEG4_TRAN_B_CALIB_VAL      142427224LL

#define LEG4_TRAN_A_RAD_PER_CNT  -9.8039216e-09
#define LEG4_TRAN_B_RAD_PER_CNT   9.8039216e-09

#define LEG4_MOTOR_A_DIRECTION            -1.0
#define LEG4_MOTOR_B_DIRECTION             1.0




// Ignore the lines below, they are used for the include magic.
#ifdef INCLUDE_LEFT_LEG
#define LEFT_LEG_A_CALIB_VAL     LEG4_LEG_A_CALIB_VAL
#define LEFT_LEG_B_CALIB_VAL     LEG4_LEG_B_CALIB_VAL

#define LEFT_LEG_A_RAD_PER_CNT   LEG4_LEG_A_RAD_PER_CNT
#define LEFT_LEG_B_RAD_PER_CNT   LEG4_LEG_B_RAD_PER_CNT

#define LEFT_TRAN_A_CALIB_VAL    LEG4_TRAN_A_CALIB_VAL
#define LEFT_TRAN_B_CALIB_VAL    LEG4_TRAN_B_CALIB_VAL 

#define LEFT_TRAN_A_RAD_PER_CNT  LEG4_TRAN_A_RAD_PER_CNT
#define LEFT_TRAN_B_RAD_PER_CNT  LEG4_TRAN_B_RAD_PER_CNT

#define LEFT_MOTOR_A_DIRECTION   LEG4_MOTOR_A_DIRECTION
#define LEFT_MOTOR_B_DIRECTION   LEG4_MOTOR_B_DIRECTION
#endif

#ifdef INCLUDE_RIGHT_LEG
#define RIGHT_LEG_A_CALIB_VAL    LEG4_LEG_A_CALIB_VAL
#define RIGHT_LEG_B_CALIB_VAL    LEG4_LEG_B_CALIB_VAL

#define RIGHT_LEG_A_RAD_PER_CNT  LEG4_LEG_A_RAD_PER_CNT
#define RIGHT_LEG_B_RAD_PER_CNT  LEG4_LEG_B_RAD_PER_CNT

#define RIGHT_TRAN_A_CALIB_VAL   LEG4_TRAN_A_CALIB_VAL
#define RIGHT_TRAN_B_CALIB_VAL   LEG4_TRAN_B_CALIB_VAL 

#define RIGHT_TRAN_A_RAD_PER_CNT LEG4_TRAN_A_RAD_PER_CNT
#define RIGHT_TRAN_B_RAD_PER_CNT LEG4_TRAN_B_RAD_PER_CNT

#define RIGHT_MOTOR_A_DIRECTION  LEG4_MOTOR_A_DIRECTION
#define RIGHT_MOTOR_B_DIRECTION  LEG4_MOTOR_B_DIRECTION
#endif

#endif // LEG4_DEFINITIONS
