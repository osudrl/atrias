/*
 * gui_library.h
 *
 *  Created on: May 29, 2012
 *      Author: Michael Anderson
 */

#ifndef GUI_LIBRARY_H_
#define GUI_LIBRARY_H_

#include <gtkmm.h>
#include <ros/node_handle.h>

#ifdef __cplusplus
extern "C" {
#endif

bool guiInit(Glib::RefPtr<Gtk::Builder> gui);
void guiUpdate();
void guiTakedown();
void getParameters();
void setParameters();

#ifdef __cplusplus
}
#endif

#endif /* GUI_LIBRARY_H_ */
