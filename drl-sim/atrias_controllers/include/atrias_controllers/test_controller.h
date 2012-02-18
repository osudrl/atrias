#ifndef TEST_CONTROLLER_H
#define TEST_CONTROLLER_H

#include <atrias_controllers/controller.h>

void initialize_test_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void update_test_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void takedown_test_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

#endif // TEST_CONTROLLER_H
