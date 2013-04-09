/*
 * controller_gui.cpp
 *
 * ToSubstitutePackageName controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <ToSubstitutePackageName/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	// Call gui->get_widget("name", pointer) here.

	// Check all pointers are non-null here (return true if there's an error).

	// Set ranges and values here.

	// Set up subscriber and publisher.
	sub = nh.subscribe("controller_status", 0, controllerCallback);
	pub = nh.advertise<ToSubstitutePackageName::controller_input>("controller_input", 0);
	return false;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const ToSubstitutePackageName::controller_status &status) {
	controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
	// Get parameters in the atrias_gui namespace.
	// Configure the GUI.
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
}

//! \brief Update the GUI.
void guiUpdate() {
	// Set values in controllerDataOut variable here

	// publish the controller input
	pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}
