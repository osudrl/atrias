/*
 * controller_gui.cpp
 *
 * atc_hip_force_test controller
 *
 *  Created on: Dec 19, 2012
 *      Author: Ryan Van Why
 */

#include <atc_hip_force_test/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	gui->get_widget("torque_A_hscale", torque_A_hscale);
	gui->get_widget("torque_B_hscale", torque_B_hscale);
	gui->get_widget("torque_hip_hscale", torque_hip_hscale);

	if (torque_A_hscale && torque_B_hscale && torque_hip_hscale) {
		// Set ranges.
		torque_A_hscale->set_range(-10., 10.);
		torque_B_hscale->set_range(-10., 10.);
		torque_hip_hscale->set_range(-10., 10.);

		// Set up subscriber and publisher.
		sub = nh.subscribe("atc_hip_force_test_status", 0, controllerCallback);
		pub = nh.advertise<atc_hip_force_test::controller_input>("atc_hip_force_test_input", 0);
		return true;
	}
	return false;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_hip_force_test::controller_status &status) {
	controllerDataIn = status;
}

//! \brief Update the GUI.
void guiUpdate() {
	controllerDataOut.des_motor_torque_A   = torque_A_hscale->get_value();
	controllerDataOut.des_motor_torque_B   = torque_B_hscale->get_value();
	controllerDataOut.des_motor_torque_hip = torque_hip_hscale->get_value();
	pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

// vim: noexpandtab
