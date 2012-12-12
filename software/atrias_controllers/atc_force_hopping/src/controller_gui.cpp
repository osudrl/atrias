/*
 * controller_gui.cpp
 *
 * atc_force_hopping controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <atc_force_hopping/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	gui->get_widget("flightLegLen",    flightLegLen);
	gui->get_widget("retractDiff",     retractDiff);
	gui->get_widget("flightP",         flightP);
	gui->get_widget("flightD",         flightD);
	gui->get_widget("stanceP",         stanceP);
	gui->get_widget("stanceD",         stanceD);
	gui->get_widget("stancePSrcCombo", stancePSrc);
	gui->get_widget("stanceDSrcCombo", stanceDSrc);
	gui->get_widget("shutdownBtn",     shutdownBtn);

	if (!flightLegLen ||
	    !retractDiff  ||
	    !flightP      ||
	    !stanceD      ||
	    !flightP      ||
	    !stanceD      ||
	    !stancePSrc   ||
	    !stanceDSrc   ||
	    !shutdownBtn) {
	
		// Oops, GUI construction failure...
		return false;
	}

	// Set ranges.
	torque_A_hscale->set_range(-10., 10.);
	torque_B_hscale->set_range(-10., 10.);
	torque_hip_hscale->set_range(-10., 10.);

	// Set up subscriber and publisher.
	sub = nh.subscribe("atc_force_hopping_status", 0, controllerCallback);
	pub = nh.advertise<atc_force_hopping::controller_input>("atc_force_hopping_input", 0);
	return true;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_force_hopping::controller_status &status) {
	controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
	// Get parameters in the atrias_gui namespace.
	nh.getParam("/atrias_gui/torque_A", torque_A_param);
	nh.getParam("/atrias_gui/torque_B", torque_B_param);
	nh.getParam("/atrias_gui/torque_hip", torque_hip_param);

	// Configure the GUI.
	torque_A_hscale->set_value(torque_A_param);
	torque_B_hscale->set_value(torque_B_param);
	torque_hip_hscale->set_value(torque_hip_param);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
	nh.setParam("/atrias_gui/torque_A", torque_A_param);
	nh.setParam("/atrias_gui/torque_B", torque_B_param);
	nh.setParam("/atrias_gui/torque_hip", torque_hip_param);
}

//! \brief Update the GUI.
void guiUpdate() {
	controllerDataOut.des_motor_torque_A   = torque_A_param   = torque_A_hscale->get_value();
	controllerDataOut.des_motor_torque_B   = torque_B_param   = torque_B_hscale->get_value();
	controllerDataOut.des_motor_torque_hip = torque_hip_param = torque_hip_hscale->get_value();
	pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

// vim: noexpandtab
