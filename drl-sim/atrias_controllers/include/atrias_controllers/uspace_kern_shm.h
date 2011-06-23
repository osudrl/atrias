// Devin Koepl

#ifndef FUNCS_H_USPACE_KERN_SHM
#define FUNCS_H_USPACE_KERN_SHM

#include <atrias_controllers/controller.h>

#define SHM_TO_KERN_KEY			5000
#define SHM_TO_USPACE_KEY		5001
#define SHM_TO_USPACE_ENTRIES	1000000

// Messages
#define SHM_TO_USPACE_MSGS		100

#define NO_MSG								0
#define INFO_MSG							1
#define WARN_MSG							2
#define ERROR_MSG							3

// Data to the kernel.  We can just use a spin lock for data going to the kernel.
typedef struct
{
	// Index where kernel is reading.
	unsigned char		index;
	unsigned char		req_switch;

	// To Controller
	unsigned char		command[2];
	unsigned char		controller_requested[2];
	unsigned char		controller_data[2][SIZE_OF_CONTROLLER_DATA];
} ToKernShm;

// This is the data that should be logged.  Each struct represents the state of the system
// at a discrete time.  This is implemented as a ring buffer.
typedef struct
{
	int index;

	ControllerInput 	controller_input[SHM_TO_USPACE_ENTRIES];
	ControllerOutput 	controller_output[SHM_TO_USPACE_ENTRIES]; 
	ControllerState		controller_state[SIZE_OF_CONTROLLER_STATE_DATA];

	// Messages
	unsigned char		msg_index;  // Index of the last written message.
	unsigned char		msg_priority[SHM_TO_USPACE_MSGS];
	unsigned char		msg[SHM_TO_USPACE_MSGS][100];
} ToUspaceShm;

#endif // FUNCS_H_USPACE_KERN_SHM
