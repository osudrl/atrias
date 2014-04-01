/*
 * controller_gui.cpp
 *
 * atc_stabilized_standing controller
 *
 *  Created on: April 1, 2014
 *      Author: Mikhail S. Jones
 */

#include <atc_stabilized_standing/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	// Main controller options
	gui->get_widget("main_controller_combobox", main_controller_combobox);

	// Set up subscriber and publisher.
	sub = nh.subscribe("ATCStabilizedStanding_status", 0, controllerCallback);
	pub = nh.advertise<atc_stabilized_standing::controller_input>("ATCStabilizedStanding_input", 0);
	return true;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_stabilized_standing::controller_status &status) {
	controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
	// Main controller options
	int main_controller;
	nh.getParam("/atrias_gui/main_controller", main_controller);
	controllerDataOut.main_controller = (uint8_t)main_controller;
	main_controller_combobox->set_active(controllerDataOut.main_controller);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
	// Main controller options
	nh.setParam("/atrias_gui/main_controller", controllerDataOut.main_controller);
}

//! \brief Update the GUI.
void guiUpdate() {
	// Main controller options
	controllerDataOut.main_controller = (uint8_t)main_controller_combobox->get_active_row_number();

	// publish the controller input
	pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}
