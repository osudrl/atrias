#ifndef LEG_TORQUE_CONTROLLER_H
#define LEG_TORQUE_CONTROLLER_H

#include <atrias_controllers/controller.h>

void initialize_leg_torque_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void update_leg_torque_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void takedown_leg_torque_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

#endif // LEG_TORQUE_CONTROLLER_H
