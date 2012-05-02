// Devin Koepl

#ifndef FUNCS_H_CONTROL_SWITCHER_STATE_MACHINE
#define FUNCS_H_CONTROL_SWITCHER_STATE_MACHINE

#ifndef COMPILE_FOR_RTAI
#include <stdint.h>
#else
#include <rtai_sem.h>
#endif

#include <atrias/ucontroller.h>

#include <atrias_controllers/controller.h>

// Control switcher state machine (CSSM) states.
#define CSSM_STATE_DISABLED		0
#define CSSM_STATE_ERROR		1
#define CSSM_STATE_ENABLED	 	2
#define CSSM_STATE_INIT		 	3
#define CSSM_STATE_FINI		 	4

//==================================================================================//

// These all pretty much have to be global since we have to call them from inside an object in the Gazebo namespace,
// and from our RTAI kernel module.

// Control function pointers.

static void (*initialize_controller)(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
static void (*update_controller)(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
static void (*takedown_controller)(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

// Control function prototypes.

extern void initialize_no_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void update_no_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void takedown_no_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

extern void initialize_motor_torque_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void update_motor_torque_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void takedown_motor_torque_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

extern void initialize_motor_position_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void update_motor_position_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void takedown_motor_position_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

extern void initialize_leg_torque_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void update_leg_torque_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void takedown_leg_torque_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

extern void initialize_leg_position_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void update_leg_position_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void takedown_leg_position_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

extern void initialize_leg_force_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void update_leg_force_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void takedown_leg_force_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

extern void initialize_leg_angle_sin_wave(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void update_leg_angle_sin_wave(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void takedown_leg_angle_sin_wave(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

extern void initialize_raibert_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void update_raibert_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void takedown_raibert_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

extern void initialize_test_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void update_test_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void takedown_test_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

extern void initialize_demo_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void update_demo_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
extern void takedown_demo_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
//==================================================================================//

extern void control_switcher_state_machine(ControllerInput *, ControllerOutput *, ControllerState *, ControllerData *);

extern void switch_controllers(ControllerState *, ControllerData *);

//extern unsigned char get_state();

#endif // FUNCS_H_CONTROL_SWITCHER_STATE_MACHINE
