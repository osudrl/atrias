/*
 * controller_gui.cpp
 *
 * atc_velocity_tuning controller
 *
 *  Created on: May 5, 2012
 *      Author: Ryan Van Why
 */

#include <atc_velocity_tuning/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	gui->get_widget("minPosBox",    minPosSpnBtn);
	gui->get_widget("maxPosBox",    maxPosSpnBtn);
	gui->get_widget("desVelBox",    desVelSpnBtn);
	gui->get_widget("kpBox",        kpSpnBtn);
	gui->get_widget("freqBox",      freqSpnBtn);
	gui->get_widget("sensorButton", sensorToggle);
	gui->get_widget("absEncoder",   absEncoder);
	gui->get_widget("incEncoder",   incEncoder);
	gui->get_widget("forward",      forward);
	gui->get_widget("reverse",      reverse);
	gui->get_widget("automatic",    automatic);
	gui->get_widget("haltButton",   haltToggle);

	if (!minPosSpnBtn ||
	    !maxPosSpnBtn ||
	    !desVelSpnBtn ||
	    !kpSpnBtn     ||
	    !sensorToggle ||
	    !absEncoder   ||
	    !incEncoder   ||
	    !forward      ||
	    !reverse      ||
	    !haltToggle) {
		
		return false;
	}
	
	// Set ranges.
	minPosSpnBtn->set_range(0.0, 1.5);
	maxPosSpnBtn->set_range(0.0, 1.5);
	desVelSpnBtn->set_range(0.0, 10.0);
	kpSpnBtn->set_range(0.0, 300.0);
	
	minPosSpnBtn->set_value(.2);
	maxPosSpnBtn->set_value(.4);
	desVelSpnBtn->set_value(1.0);
	kpSpnBtn->set_value(10.0);

	absEncoder->set_active(true);

	// Set up publisher.
	pub = nh.advertise<atc_velocity_tuning::controller_input>("atc_velocity_tuning_input", 0);
	return true;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
	return;
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
	return;
}

//! \brief Update the GUI.
void guiUpdate() {
	double minPos = minPosSpnBtn->get_value();
	double maxPos = maxPosSpnBtn->get_value();
	controllerDataOut.minPos    = MIN(minPos, maxPos);
	controllerDataOut.maxPos    = MAX(minPos, maxPos);
	controllerDataOut.desVel    = desVelSpnBtn->get_value();
	controllerDataOut.Kp        = kpSpnBtn->get_value();
	controllerDataOut.sensor    = sensorToggle->get_active() ? 1 : 0;
	controllerDataOut.autoFrequency = freqSpnBtn->get_value();
	// The following ternary tree simply sets the value in
	// relayMode to the relevant int for the ticked radio
	// button, as defined in controller_input.msg
	controllerDataOut.relayMode = (absEncoder->get_active()) ? 0 :
	                              (incEncoder->get_active()) ? 1 :
	                              (forward->get_active())    ? 2 :
							      (reverse->get_active())    ? 3 :
								                               4; // 4 == automatic
	controllerDataOut.halt = haltToggle->get_active();
	pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

