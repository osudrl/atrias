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

// ATRIAS
#include <atrias/ucontroller.h>

// ATRIAS controller stuff
#include <atrias_controllers/controller.h>
//#include <atrias_controller_switcher/control_switcher_state_machine.h>

// DRL Library
#include <drl_library/discretize.h>
#include <drl_library/drl_math.h>

#define NUM_OF_MEDULLAS_ON_ROBOT            4
#define NUM_OF_SLAVES_IN_SIMULATION_MACHINE 1

#define A_INDEX    1
#define B_INDEX    0
#define HIP_INDEX  2
#define BOOM_INDEX 3

// Horizontal velocity windowing filter parameters.
#define HOR_VEL_FILTER_EPS                  0.003
#define HOR_VEL_WINDOW                      100

/*****************************************************************************/

Shm shm;

unsigned char initialize_shm( void );
//void takedown_shm( void );

void control_wrapper_state_machine( uControllerInput **, uControllerOutput ** );


unsigned char state_wakeup( uControllerInput **, uControllerOutput **, unsigned char );
unsigned char state_restart( uControllerInput **, uControllerOutput **, unsigned char );
unsigned char state_check( uControllerInput **, uControllerOutput **, unsigned char );
unsigned char state_initialize(  uControllerInput **, uControllerOutput **, unsigned char );
unsigned char state_run( uControllerInput **, uControllerOutput **, unsigned char );
unsigned char state_error( uControllerInput **, uControllerOutput **, unsigned char );

/*****************************************************************************/

#endif // ATRIAS_CONTROLLER_WRAPPER_H
