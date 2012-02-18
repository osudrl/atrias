#ifndef MOTOR_POSITION_CONTROLLER_H
#define MOTOR_POSITION_CONTROLLER_H

#include <atrias_controllers/controller.h>

void initialize_motor_position_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void update_motor_position_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void takedown_motor_position_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

#endif // MOTOR_POSITION_CONTROLLER_H
