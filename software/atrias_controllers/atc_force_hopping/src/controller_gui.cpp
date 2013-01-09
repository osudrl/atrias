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
	gui->get_widget("flightP",         flightP);
	gui->get_widget("flightD",         flightD);
	gui->get_widget("stanceP",         stanceP);
	gui->get_widget("stanceD",         stanceD);
	gui->get_widget("hipP",            hipP);
	gui->get_widget("hipD",            hipD);
	gui->get_widget("stancePSrcCombo", stancePSrc);
	gui->get_widget("stanceDSrcCombo", stanceDSrc);
	gui->get_widget("stateLbl",        stateLbl);
	gui->get_widget("lockLeg",         lockLeg);

	if (!flightLegLen ||
	    !flightP      ||
	    !stanceD      ||
	    !flightP      ||
	    !stanceD      ||
	    !hipP         ||
	    !hipD         ||
	    !stancePSrc   ||
	    !stanceDSrc   ||
	    !stateLbl     ||
	    !lockLeg) {
	
		// Oops, GUI construction failure...
		return false;
	}

	// Set ranges.
	flightLegLen->set_range(.7, 0.95);
	flightP->set_range(     0.0, 1000.0);
	flightD->set_range(     0.0, 50.0);
	stanceP->set_range(     0.0, 3000.0);
	stanceD->set_range(     0.0, 150.0);
	hipP->set_range(        0.0, 300.0);
	hipD->set_range(        0.0, 40);
	
	flightLegLen->set_value(0.9);
	flightP->set_value(     600.0);
	flightD->set_value(     20.0);
	stanceP->set_value(     1500.0);
	stanceD->set_value(     20.0);
	hipP->set_value(        150.0);
	hipD->set_value(        10.0);

	// We also need to deal with the combo boxes, but that's for later...

	// Set up subscriber and publisher.
	sub = nh.subscribe("atc_force_hopping_status", 0, controllerCallback);
	pub = nh.advertise<atc_force_hopping::controller_input>("atc_force_hopping_input", 0);
	return true;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_force_hopping::controller_status &status) {
	controllerDataIn = status;
}

//! \brief Update the GUI.
void guiUpdate() {
	controllerDataOut.flightLegLen = flightLegLen->get_value();
	controllerDataOut.flightP      = flightP->get_value();
	controllerDataOut.flightD      = flightD->get_value();
	controllerDataOut.stanceP      = stanceP->get_value();
	controllerDataOut.stanceD      = stanceD->get_value();
	controllerDataOut.hipP         = hipP->get_value();
	controllerDataOut.hipD         = hipD->get_value();
	controllerDataOut.lockLeg      = lockLeg->get_active() ? 1 : 0;
	pub.publish(controllerDataOut);

	switch ((State) controllerDataIn.state) {
		case State::INIT:
			stateLbl->set_text("Initializing");
			break;
		case State::FLIGHT:
			stateLbl->set_text("Flight");
			break;
		case State::STANCE:
			stateLbl->set_text("Stance");
			break;
		case State::LOCKED:
			stateLbl->set_text("Locked");
			break;
	}
}

//! \brief Take down the GUI.
void guiTakedown() {
}

// vim: noexpandtab
