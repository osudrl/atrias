// Devin Koepl

#ifndef FUNCS_H_CONTROL_SWITCHER_STATE_MACHINE
#define FUNCS_H_CONTROL_SWITCHER_STATE_MACHINE

#include <stdint.h>

#include <atrias/ucontroller.h>

#include <atrias_controllers/controller.h>

// TODO: All of this will be unnecessary once we figure out how to load/unload
// controllers with roslaunch or Orocos.
#include <atrias_controllers/no_controller.h>
#include <atrias_controllers/motor_torque_controller.h>
#include <atrias_controllers/motor_position_controller.h>
#include <atrias_controllers/leg_torque_controller.h>
#include <atrias_controllers/leg_position_controller.h>
#include <atrias_controllers/leg_force_controller.h>
#include <atrias_controllers/leg_angle_sin_wave.h>
#include <atrias_controllers/raibert_controller.h>
#include <atrias_controllers/test_controller.h>

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

//==================================================================================//

extern void control_switcher_state_machine(ControllerInput *, ControllerOutput *, ControllerState *, ControllerData *);

extern void switch_controllers(ControllerState *, ControllerData *);

//extern unsigned char get_state();

#endif // FUNCS_H_CONTROL_SWITCHER_STATE_MACHINE
