#ifndef RAIBERT_CONTROLLER_H
#define RAIBERT_CONTROLLER_H

#include <atrias_controllers/controller.h>

void initialize_raibert_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void update_raibert_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void takedown_raibert_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

#endif // RAIBERT_CONTROLLER_H
