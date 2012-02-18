#ifndef LEG_FORCE_CONTROLLER_H
#define LEG_FORCE_CONTROLLER_H

#include <atrias_controllers/controller.h>

void initialize_leg_force_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void update_leg_force_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void takedown_leg_force_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

#endif // LEG_FORCE_CONTROLLER_H
