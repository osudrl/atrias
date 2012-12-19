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
	gui->get_widget("flightP", flightP);
	gui->get_widget("flightD", flightD);
	gui->get_widget("stanceP", stanceP);
	gui->get_widget("stanceD", stanceD);
	gui->get_widget("toeFilterGain", toeFilterGain);
	gui->get_widget("toeThreshold",  toeThreshold);
	gui->get_widget("legLen",        legLen);
	gui->get_widget("force",         force);

	if (!flightP ||
	    !flightD ||
	    !stanceP ||
	    !stanceD ||
	    !toeFilterGain ||
	    !toeThreshold  ||
	    !legLen        ||
	    !force) {
		// We've failed to initialize
		return false;
	}

	// Set ranges.
	flightP->set_range(0.0, 200.0);
	flightD->set_range(0.0, 15.0);
	stanceP->set_range(0.0, 1.0);
	stanceD->set_range(0.0, 30.0);
	toeFilterGain->set_range(0.0, 1.0);
	toeThreshold->set_range(0.0, 10000.0);
	legLen->set_range(0.5, 0.99);
	force->set_range(0.0, 2000.0);

	flightP->set_value(150.0);
	flightD->set_value(10.0);
	stanceP->set_value(0.0001);
	stanceD->set_value(10.0);
	toeFilterGain->set_value(0.05);
	toeThreshold->set_value(500.0);
	legLen->set_value(0.9);
	force->set_value(100.0);

	// Set up subscriber and publisher.
	sub = nh.subscribe("atc_hip_force_test_status", 0, controllerCallback);
	pub = nh.advertise<atc_hip_force_test::controller_input>("atc_hip_force_test_input", 0);
	return true;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_hip_force_test::controller_status &status) {
	controllerDataIn = status;
}

//! \brief Update the GUI.
void guiUpdate() {
//	controllerDataOut.des_motor_torque_A   = torque_A_hscale->get_value();
//	controllerDataOut.des_motor_torque_B   = torque_B_hscale->get_value();
//	controllerDataOut.des_motor_torque_hip = torque_hip_hscale->get_value();
	pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

// vim: noexpandtab
