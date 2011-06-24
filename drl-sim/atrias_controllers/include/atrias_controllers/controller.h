// Devin Koepl


#ifndef FUNCS_H_CONTROLLER
#define FUNCS_H_CONTROLLER

#include <drl_library/drl_math.h>

#ifdef COMPILE_FOR_RTAI
 #include <linux/types.h>
 #include <rtai_math.h>
#else
 #include <stdint.h>
 #include <math.h>
#endif


#define SIZE_OF_CONTROLLER_DATA 		100
#define SIZE_OF_CONTROLLER_STATE_DATA	100

//======================================================//

// Debugging print statements.

//#define DEBUG_CONTROLLERS
#undef	DEBUG_CONTROLLERS


#ifdef DEBUG_CONTROLLERS
	#ifdef COMPILE_FOR_RTAI
			//#include <rtai_sem.h>
			#define PRINT_MSG	rt_printk
			#define PRINT_WARN	rt_printk
			//#define PRINT_MSG	rtai_print_to_screen
			//#define PRINT_WARN	rtai_print_to_screen
		#else
			#include <ros/ros.h>
			#define PRINT_MSG	ROS_INFO
			#define PRINT_WARN	ROS_WARN
	#endif
#else
	#define PRINT_MSG	// printf
	#define PRINT_WARN	// printf
#endif

// Temporary for testing in uspace.
//#include <rtai_lxrt.h>

//#define PRINT_MSG rtai_print_to_screen
//#define PRINT_WARN rtai_print_to_screen

//======================================================//

// Controller types available.

#define NO_CONTROLLER 					0
#define MOTOR_TORQUE_CONTROLLER 		1
#define MOTOR_POSITION_CONTROLLER 		2
#define LEG_TORQUE_CONTROLLER			3
#define LEG_POSITION_CONTROLLER 		4
#define SINE_WAVE_CONTROLLER			5
#define RAIBERT_CONTROLLER				6
#define EQU_GAIT_CONTROLLER 7
#define TEST_CONTROLLER 8

//======================================================//

// If the controller commands both motors to have a torque below this value, assume that no controller is present,
// and command a small torque to keep the robot off of its hardstops.
#define MIN_TRQ_THRESH					1E-9

//======================================================//

// General control structs.

typedef struct
{
	float 			body_angle;
	float			motor_angleA;
	float			motor_angleB;
	float			leg_angleA;
	float			leg_angleB;

	float			body_ang_vel;
	float			motor_velocityA;
	float 			motor_velocityB;
	float			leg_velocityA;
	float 			leg_velocityB;

	float			height;

	float 			horizontal_velocity;
	float 			vertical_velocity;

	unsigned char	motor_currentA;
	unsigned char	motor_currentB;

	unsigned char	toe_switch;
	
	unsigned char	command;
} ControllerInput;

typedef struct
{
	float			motor_torqueA;
	float			motor_torqueB;
} ControllerOutput;

// This struct is where the controller can keep personal information.
typedef struct
{
	unsigned char 	state;
	unsigned char 	controller_loaded;

	// Controller specific space.
	unsigned char 	data[SIZE_OF_CONTROLLER_STATE_DATA];
} ControllerState;

// This struct is the input to the controller.
typedef struct
{
  unsigned char command;
  unsigned char controller_requested;

  // Controller specific space.
  unsigned char data[SIZE_OF_CONTROLLER_DATA];
} ControllerData;

//======================================================//

// Specific control structs

typedef struct
{
  float mtr_trqA;
  float mtr_trqB;
} MtrTrqControllerData;

typedef struct
{
  float mtr_angA;
  float mtr_angB;
  float p_gain;
  float d_gain;
} MtrPosControllerData;

typedef struct
{
  float leg_ang_trq;
  float leg_len_trq;
} LegTrqControllerData;

typedef struct
{
  float leg_ang;
  float leg_len;
  float p_gain;
  float d_gain;
} LegPosControllerData;

typedef struct
{
  // Inputs
  float leg_ang_frq;
  float leg_ang_amp;
  float leg_len_frq;
  float leg_len_amp;
  float p_gain;
  float d_gain;

  // For Control
  float time;
} SinWaveControllerData;

typedef struct
{
  float time;
} SinWaveControllerState;

typedef struct
{
  // Inputs
  float des_hor_vel;
  float des_hop_ht;
  float hor_vel_gain;
  float hop_ht_gain;
  float leg_ang_gain;
  float stance_p_gain;
  float stance_d_gain;
  float stance_spring_threshold;
  float preferred_leg_len;
  float flight_p_gain;
  float flight_d_gain;
  float flight_spring_threshold;
} RaibertControllerData;

typedef struct
{
  unsigned char in_flight;
  unsigned char after_mid_stance;

  float peak_ht;
  float last_leg_len;
} RaibertControllerState;

typedef struct
{
  // Nothing
} TestControllerData;

typedef struct
{
  unsigned char in_flight;
} TestControllerState;

// Macros for dereferencing pointers.
#define MTR_TRQ_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((MtrTrqControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define MTR_POS_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((MtrPosControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define LEG_TRQ_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((LegTrqControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define LEG_POS_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((LegPosControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define SIN_WAVE_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((SinWaveControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define RAIBERT_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((RaibertControllerData *)(&(CONTROLLER_DATA_PTR->data)))
#define TEST_CONTROLLER_DATA(CONTROLLER_DATA_PTR) ((TestControllerData *)(&(CONTROLLER_DATA_PTR->data)))

#define MTR_TRQ_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((MtrTrqControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define MTR_POS_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((MtrPosControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define LEG_TRQ_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((LegTrqControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define LEG_POS_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((LegPosControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define SIN_WAVE_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((SinWaveControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define RAIBERT_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((RaibertControllerState *)(&(CONTROLLER_STATE_PTR->data)))
#define TEST_CONTROLLER_STATE(CONTROLLER_STATE_PTR) ((TestControllerState *)(&(CONTROLLER_STATE_PTR->data)))

#endif // FUNCS_H_CONTROLLER
