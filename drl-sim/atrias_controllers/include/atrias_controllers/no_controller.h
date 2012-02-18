#ifndef NO_CONTROLLER_H
#define NO_CONTROLLER_H

#include <atrias_controllers/controller.h>

void initialize_no_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void update_no_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void takedown_no_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

#endif // NO_CONTROLLER_H
