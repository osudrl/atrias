#ifndef FUNCS_H_ECAT_INTERFACE
#define FUNCS_H_ECAT_INTERFACE

#include <atrias/ucontroller.h>

#define SHMSZ     			27
#define GAZEBO_SHM_KEY 	5678

#define NUM_OF_MEDULLAS 3

typedef struct
{
	uControllerInput ucontroller_inputA;
	uControllerInput ucontroller_inputB;
	uControllerInput ucontroller_input_hip;
} AtriasInput;

typedef struct
{
	uControllerOutput ucontroller_outputA;
	uControllerOutput ucontroller_outputB;
	uControllerOutput ucontroller_output_hip;
} AtriasOutput;

typedef struct
{
	uint8_t		 lock;
	uint8_t		 fresh;

	AtriasInput atrias_input;

	AtriasOutput atrias_output;
} SimulationEcatInterface;

#endif // FUNCS_H_ECAT_INTERFACE
