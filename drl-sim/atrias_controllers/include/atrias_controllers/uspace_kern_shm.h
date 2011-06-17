// Devin Koepl

#ifndef FUNCS_H_USPACE_KERN_SHM
#define FUNCS_H_USPACE_KERN_SHM

#include <atrias_controllers/controller.h>

#define SHM_TO_KERN_KEY			5000
#define SHM_TO_USPACE_KEY		5001
#define SHM_TO_USPACE_ENTRIES	200

// Data to the kernel.  We can just use a spin lock for data going to the kernel.
typedef struct
{
	// Spin lock for managing sharing.
	unsigned char		lock;

	// To Controller
	unsigned char		command;
	unsigned char		controller_requested;
	unsigned char		controller_data[SIZE_OF_CONTROLLER_DATA];
} DataToKern;

// This is the data that should be logged.  Each struct represents the state of the system
// at a discrete time.  This is implemented as a ring buffer.
typedef struct
{
	// True when this entry has not been logged.
	unsigned char 		fresh;

	// Counter for debugging, incremented each write.
	unsigned int 		cnt;
	unsigned char		index;

	ControllerInput 	controller_input;
	ControllerOutput 	controller_output; 
} DataToUspace;

#endif // FUNCS_H_USPACE_KERN_SHM
