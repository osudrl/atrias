#ifndef DEMO_CONTROLLER_H
#define DEMO_CONTROLLER_H

#include <atrias_controllers/controller.h>

#define DEMO1 0
#define DEMO2 1
#define DEMO3 2
#define DEMO4 3

#define DEMO_STATE_STOPPED 0
#define DEMO_STATE_STARTING 1
#define DEMO_STATE_RUNNING 2
#define DEMO_STATE_STOPPING 3


typedef struct {
	float x;
	float y;
	float z;
	float x_vel;
	float y_vel;
	float z_vel;
} CartPosition;

void initialize_demo_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void update_demo_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void takedown_demo_controller(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

void start_demo(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);
void stop_demo(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*);

RobotPosition demo1(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*, float);
RobotPosition demo2(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*, float);
RobotPosition demo3(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*, float);
RobotPosition demo4(ControllerInput*, ControllerOutput*, ControllerState*, ControllerData*, float);

RobotPosition cartesianToRobot(CartPosition);

#endif // DEMO_CONTROLLER_H
