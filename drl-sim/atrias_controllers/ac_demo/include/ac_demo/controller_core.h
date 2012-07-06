/*
 * controller_core.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_CORE_H_
#define CONTROLLER_CORE_H_

#include <ac_demo/controller_common.h>
#include <atrias_control/core_library.h>
#include <math.h>
#include <drl_library/drl_math.h>

#define DEMO1 0
#define DEMO2 1
#define DEMO3 2
#define DEMO4 3
#define DEMO5 4
#define DEMO6 5

#define DEMO_STATE_STOPPED 0
#define DEMO_STATE_STARTING 1
#define DEMO_STATE_RUNNING 2
#define DEMO_STATE_STOPPING 3

#define MID_MOT_ANG_A    -1.0254
#define MID_MOT_ANG_B       4.167
#define F_SLOW .1

#define FORCE_CONTROLLER_P_GAIN 3250
#define FORCE_CONTROLLER_D_GAIN 4

#define START_TIME 2.0

uint8_t currentState;
uint8_t currentDemo;
double elapsedTime;
RobotPosition desiredPos;
RobotPosition lastDemoPos;
float desTorqueA;
float desTorqueB;

typedef struct {
    float x;
    float y;
    float z;
    float x_vel;
    float y_vel;
    float z_vel;
} CartPosition;

void start_demo(robot_state& state, InputData* id, ControllerOutput* output);
void stop_demo(robot_state& state, InputData* id, ControllerOutput* output);

RobotPosition demo1(robot_state& state, InputData* id, ControllerOutput* output, double time);
RobotPosition demo2(robot_state& state, InputData* id, ControllerOutput* output, double time);
RobotPosition demo3(robot_state& state, InputData* id, ControllerOutput* output, double time);
RobotPosition demo4(robot_state& state, InputData* id, ControllerOutput* output, double time);
RobotPosition demo5(robot_state& state, InputData* id, ControllerOutput* output, double time);
RobotPosition demo6(robot_state& state, InputData* id, ControllerOutput* output, double time);

RobotPosition cartesianToRobot(CartPosition);
RobotPosition setTorques(robot_state* state, InputData* id, ControllerOutput* output);

#endif /* CONTROLLER_CORE_H_ */
