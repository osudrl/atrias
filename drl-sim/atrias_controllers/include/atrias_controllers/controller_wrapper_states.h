// Devin Koepl

/*****************************************************************************/

#ifndef FUNCS_H__CONTROL_WRAPPER_STATES
#define FUNCS_H_CONTROL_WRAPPER_STATES

#include <stdint.h>

// RTAI
#include <rtai_lxrt.h>

// ATRIAS
#include <atrias/ucontroller.h>

// ATRIAS Controllers
#include <atrias_controllers/controller.h>
#include <atrias_controllers/control_switcher_state_machine.h>

// DRL Library
#include <drl_library/discretize.h>
#include <drl_library/drl_math.h>

#define WAKE_UP_STATE 						0
#define RESTART_STATE 						1
#define CHECK_STATE 						2
#define INITIALIZE_STATE 					3
#define RUN_STATE 							4
#define ERROR_STATE 						5

#define NUM_OF_MEDULLAS_ON_ROBOT 			4 
#define NUM_OF_SLAVES_IN_SIMULATION_MACHINE 1

#define	A_INDEX								0
#define B_INDEX								1
#define HIP_INDEX							2
#define BOOM_INDEX							3

// Horizontal velocity windowing filter parameters.
#define HOR_VEL_FILTER_EPS					0.003
#define HOR_VEL_WINDOW						100

#define SIZE_OF_DATA_RING_BUFFER			1000000

/*****************************************************************************/

void control_wrapper_state_machine( uControllerInput **, uControllerOutput ** );

unsigned char wake_up_state( uControllerInput **, uControllerOutput **, unsigned char );
unsigned char restart_state( uControllerInput **, uControllerOutput **, unsigned char );
unsigned char check_state( uControllerInput **, uControllerOutput **, unsigned char );
unsigned char initialize_state(  uControllerInput **, uControllerOutput **, unsigned char );
unsigned char run_state( uControllerInput **, uControllerOutput **, unsigned char );
unsigned char error_state( uControllerInput **, uControllerOutput **, unsigned char );

// Controller structs.
static ControllerInput 		controller_input[SIZE_OF_DATA_RING_BUFFER];
static ControllerOutput 	controller_output[SIZE_OF_DATA_RING_BUFFER];
static ControllerState 		controller_state[2];
static ControllerData 		controller_data[2];

static unsigned int			io_index = 0;
static unsigned char		state_index = 0;
static unsigned char		data_index = 0;

/*****************************************************************************/

#endif // FUNCS_H_CONTROL_WRAPPER_STATES
