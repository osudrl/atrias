#ifndef LEG_ANGLE_SIN_WAVE_H
#define LEG_ANGLE_SIN_WAVE_H

#include <atrias_controllers/controller.h>

void initialize_leg_angle_sin_wave(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void update_leg_angle_sin_wave(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void takedown_leg_angle_sin_wave(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

#endif // LEG_ANGLE_SIN_WAVE_H
