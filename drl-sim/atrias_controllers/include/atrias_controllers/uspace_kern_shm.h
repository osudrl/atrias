// Devin Koepl

#ifndef FUNCS_H_USPACE_KERN_SHM
#define FUNCS_H_USPACE_KERN_SHM

#include <atrias_controllers/controller.h>

#define SHM_NAME				"SHM_NAME"
#define SHM_TO_USPACE_ENTRIES	1000000

// Messages
#define SHM_TO_USPACE_MSGS		100

#define NO_MSG					0
#define INFO_MSG				1
#define WARN_MSG				2
#define ERROR_MSG				3

// Data to the kernel.  We can just use a spin lock for data going to the kernel.
typedef struct
{
	// To kernel space

	// Index where kernel is reading.
	unsigned char		control_index;
	unsigned char		req_switch;

	// To Controller
	ControllerData		controller_data[2];

	// To userspace

	int 				io_index;

	ControllerInput 	controller_input[SHM_TO_USPACE_ENTRIES];
	ControllerOutput 	controller_output[SHM_TO_USPACE_ENTRIES]; 
	ControllerState		controller_state;

	// Messages
	unsigned char		msg_index;
	unsigned char		msg_priority[SHM_TO_USPACE_MSGS];
	unsigned char		msg[SHM_TO_USPACE_MSGS][100];

} Shm;

#endif // FUNCS_H_USPACE_KERN_SHM
