#ifndef MOTOR_TORQUE_CONTROLLER_H
#define MOTOR_TORQUE_CONTROLLER_H

#include <atrias_controllers/controller.h>

void initialize_motor_torque_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void update_motor_torque_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void takedown_motor_torque_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

#endif // MOTOR_TORQUE_CONTROLLER_H
