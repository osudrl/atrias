/*
 * controller_gui.cpp
 *
 * atc_deadbeat_control controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <atc_deadbeat_control/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	// Call gui->get_widget("name", pointer) here.

	// Check all pointers are non-null here (return true if there's an error).

	// Set ranges and values here.

	// Set up subscriber and publisher.
	sub = nh.subscribe("ATCDeadbeatControl_status", 0, controllerCallback);
	pub = nh.advertise<atc_deadbeat_control::controller_input>("ATCDeadbeatControl_input", 0);
	return true;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_deadbeat_control::controller_status &status) {
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
