/**
 * @file
 * @author Devin Koepl
 * @brief does all the includes for controller_wrapper_states.c
 *
 *
 */

/*****************************************************************************/

#ifndef ATRIAS_CONTROLLER_WRAPPER_H
#define ATRIAS_CONTROLLER_WRAPPER_H

#include <stdint.h>

// ATRIAS
#include <atrias_control/ucontroller.h>

// ATRIAS controller stuff
#include <atrias_control/controller.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_controller_switcher/control_switcher_state_machine.h>

// DRL Library
#include <drl_library/discretize.h>
#include <drl_library/drl_math.h>

#define NUM_OF_MEDULLAS_ON_ROBOT            3
#define NUM_OF_SLAVES_IN_SIMULATION_MACHINE 1

#define A_INDEX    1
#define B_INDEX    0
#define HIP_INDEX  2
#define BOOM_INDEX 3

// Horizontal velocity windowing filter parameters.
#define HOR_VEL_FILTER_EPS                  0.003
#define HOR_VEL_WINDOW                      100

/******************************************************************************/

#define STATE_IDLE          0
#define STATE_INIT          1
#define STATE_RUN           2
#define STATE_ERROR         3

/*****************************************************************************/

// Control wrapper variables.

unsigned short int  tranA_off;
unsigned short int  tranB_off;

int                 boom_pan_off  = 0;
int                 boom_tilt_off = 0;

unsigned short int  last_boom_pan_cnt;
unsigned short int  first_boom_pan_cnt;
unsigned short int  last_boom_tilt_cnt;

float               boom_pan_angle  = 0.;
float               boom_tilt_angle = 0.;

float               last_boom_pan_angle = 0.;
float               last_body_angle = 0.;
int32_t             body_angle_cnt_base = 0;
int32_t             last_body_cnt = 0;
float               last_body_pitch = 0.;

unsigned short int  last_tranA_cnt;
unsigned short int  last_tranB_cnt;

float               last_motor_angleA = 0.;
float               last_motor_angleB = 0.;
float               last_leg_angleA   = 0.;
float               last_leg_angleB   = 0.;
float               last_hip_angle    = 0.;
uint32_t            hip_base_count    = 0;

float               legAoffset = 0.0;
float               legBoffset = 0.0;
int                 averageCounter = 0;

float               hor_vel;
float               hor_vel_buffer[HOR_VEL_WINDOW];
int                 hor_vel_index     = 0;

float               xPositionBase = 0;
int32_t             last_xPositionEncoder = 0;

float               last_zPosition = 0;
float               last_xPosition = 0;

float               leg_angle;
float               leg_length;

int                 motorAinc_offset_ticks = 0;
int                 motorBinc_offset_ticks = 0;
float               motorAinc_offset_angle = 0.0;
float               motorBinc_offset_angle = 0.0;
uint32_t            last_medulla_time[NUM_OF_MEDULLAS_ON_ROBOT];
uint32_t            current_medulla_time[NUM_OF_MEDULLAS_ON_ROBOT];
uint32_t            medulla_bad_timestamp_counter[NUM_OF_MEDULLAS_ON_ROBOT];

uint32_t            buffer[4][128];
int                 bufferPos[4];
uint64_t            encAverage[4];

int                 firstRun = 0;

/*****************************************************************************/

ControllerWrapperData cwd;

void control_wrapper_state_machine( uControllerInput **, uControllerOutput ** );

unsigned char state_wakeup( uControllerInput **, uControllerOutput ** );
unsigned char state_restart( uControllerInput **, uControllerOutput **, unsigned char );
unsigned char state_check( uControllerInput **, uControllerOutput **, unsigned char );
unsigned char state_initialize(  uControllerInput **, uControllerOutput **, unsigned char );
unsigned char state_run( uControllerInput **, uControllerOutput **, unsigned char );
unsigned char state_error( uControllerInput **, uControllerOutput **, unsigned char );

/*****************************************************************************/

#endif // ATRIAS_CONTROLLER_WRAPPER_H
