/*
 * core_library.h
 *
 *  Created on: May 29, 2012
 *      Author: Michael Anderson
 */

#ifndef CORE_LIBRARY_H_
#define CORE_LIBRARY_H_

#include <atrias_control/controller.h>

#ifdef __cplusplus
extern "C" {
#endif

ControllerInitResult controllerInit();
void controllerUpdate(robot_state state, ByteArray input, ControllerOutput* output, ByteArray &status);
void controllerTakedown();

#ifdef __cplusplus
}
#endif

#endif /* CORE_LIBRARY_H_ */
