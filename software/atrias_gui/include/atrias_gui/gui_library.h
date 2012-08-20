/*
 * gui_library.h
 *
 *  Created on: May 29, 2012
 *      Author: Michael Anderson
 */

#ifndef GUI_LIBRARY_H_
#define GUI_LIBRARY_H_

#include <atrias_control/controller.h>
#include <gtkmm.h>

#ifdef __cplusplus
extern "C" {
#endif

ControllerInitResult guiInit(Glib::RefPtr<Gtk::Builder> gui);
void guiUpdate(robot_state state, ByteArray controllerStatus, ByteArray &controllerInput);
void guiStandby(robot_state state);
void guiTakedown();

#ifdef __cplusplus
}
#endif

#endif /* GUI_LIBRARY_H_ */
