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

// Checks if the given GTK handle is defined, and print out a message if not.
// Returns true if there's been an error, false if not
#define CHECK_WIDGET(name) (!(name) && (std::cout << "Widget " << #name << " not initialized!" << std::endl, true))

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
