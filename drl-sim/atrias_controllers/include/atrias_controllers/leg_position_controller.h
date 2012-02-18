#ifndef LEG_POSITION_CONTROLLER_H
#define LEG_POSITION_CONTROLLER_H

#include <atrias_controllers/controller.h>

#define DELTA_Y (0.5*sin(input->leg_angleB) + 0.5*sin(input->leg_angleA))
#define DELTA_X (-0.5*cos(input->leg_angleB) - 0.5*cos(input->leg_angleA))
#define LEG_LENGTH sqrt(DELTA_Y*DELTA_Y + DELTA_X*DELTA_X)

#define Lab 0.15557

void initialize_leg_position_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void update_leg_position_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void takedown_leg_position_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

#endif // LEG_POSITION_CONTROLLER_H
