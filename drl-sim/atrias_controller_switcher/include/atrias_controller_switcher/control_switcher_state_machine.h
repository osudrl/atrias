// Devin Koepl

#ifndef FUNCS_H_CONTROL_SWITCHER_STATE_MACHINE
#define FUNCS_H_CONTROL_SWITCHER_STATE_MACHINE

#include <stdint.h>
#include <dlfcn.h>

#include <ros/package.h>

#include <atrias_control/ucontroller.h>
#include <atrias_control/controller.h>

#include <atrias_msgs/robot_state.h>
#include <atrias_msgs/controller_input.h>
#include <atrias_msgs/controller_status.h>

#include <drl_library/drl_math.h>

#include <atrias_control/controller_metadata.h>

//==================================================================================//

// These all pretty much have to be global since we have to call them from inside an object in the Gazebo namespace,
// and from our RTAI kernel module.

// Control function pointers.

ControllerInitResult (*initialize_controller)();
void (*update_controller)(robot_state, ByteArray, ControllerOutput*, ByteArray&);
void (*takedown_controller)();

//==================================================================================//

extern void init();
extern void control_switcher_state_machine(controller_input&, controller_status&);
extern void switch_controllers(std::string name);

bool byteArraysInitialized;
ByteArray cInput; //Stores controller inputs from controller plugins
ByteArray cStatus; //Stores controller status from controller plugins

bool controller_loaded;
void* controller_handle;
std::string currentController;
uint8_t state = CSSM_STATE_INIT;

#endif // FUNCS_H_CONTROL_SWITCHER_STATE_MACHINE
