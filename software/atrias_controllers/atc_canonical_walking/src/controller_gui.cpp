/*
 * controller_gui.cpp
 *
 * atc_canonical_walking controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <atc_canonical_walking/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	// Get widgets
	gui->get_widget("leg_pos_kp_spinbutton", leg_pos_kp_spinbutton);
	gui->get_widget("leg_pos_kd_spinbutton", leg_pos_kd_spinbutton);
	gui->get_widget("hip_pos_kp_spinbutton", hip_pos_kp_spinbutton);
	gui->get_widget("hip_pos_kd_spinbutton", hip_pos_kd_spinbutton);
	gui->get_widget("main_controller_combobox", main_controller_combobox);
	gui->get_widget("left_toe_pos_spinbutton", left_toe_pos_spinbutton);
	gui->get_widget("right_toe_pos_spinbutton", right_toe_pos_spinbutton);
	gui->get_widget("tau_control_chkbtn",       tau_control_checkbutton);
	gui->get_widget("tau_hscale",               tau_hscale);
	gui->get_widget("cur_limit_spinbutton",     cur_limit_spinbutton);

	// Set ranges
	leg_pos_kp_spinbutton->set_range(0.0, 7500.0);
	leg_pos_kd_spinbutton->set_range(0.0, 500.0);
	hip_pos_kp_spinbutton->set_range(0.0, 500.0);
	hip_pos_kd_spinbutton->set_range(0.0, 50.0);
	left_toe_pos_spinbutton->set_range(2.1, 2.5);
	right_toe_pos_spinbutton->set_range(2.1, 2.5);
	tau_hscale->set_range(0.0, 1.0);
	cur_limit_spinbutton->set_range(0.0, std::max(-1*MIN_MTR_CURRENT_CMD, MAX_MTR_CURRENT_CMD));

	// Set increments
	leg_pos_kp_spinbutton->set_increments(10.0, 0.0);
	leg_pos_kd_spinbutton->set_increments(1.0, 0.0);
	hip_pos_kp_spinbutton->set_increments(10.0, 0.0);
	hip_pos_kd_spinbutton->set_increments(1.0, 0.0);
	left_toe_pos_spinbutton->set_increments(0.01, 0.0);
	right_toe_pos_spinbutton->set_increments(0.01, 0.0);
	tau_hscale->set_increments(.01, 0.0);
	cur_limit_spinbutton->set_increments(1.0, 0.0);

	// Set values
	leg_pos_kp_spinbutton->set_value(2000.0);
	leg_pos_kd_spinbutton->set_value(20.0);
	hip_pos_kp_spinbutton->set_value(150.0);
	hip_pos_kd_spinbutton->set_value(10.0);
	left_toe_pos_spinbutton->set_value(2.20);
	right_toe_pos_spinbutton->set_value(2.45);
	tau_control_checkbutton->set_active(false);
	tau_hscale->set_value(0.0);
	cur_limit_spinbutton->set_value(7.0);

	// Set up subscriber and publisher.
	sub = nh.subscribe("ATCCanonicalWalking_status", 0, controllerCallback);
	pub = nh.advertise<atc_canonical_walking::controller_input>("ATCCanonicalWalking_input", 0);
	return true;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_canonical_walking::controller_status &status) {
	controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
	// Get parameters in the atrias_gui namespace.
	nh.getParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
	nh.getParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
	nh.getParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
	nh.getParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
	int main_controller;
	nh.getParam("/atrias_gui/main_controller", main_controller);
	controllerDataOut.main_controller = (uint8_t)main_controller;
	nh.getParam("/atrias_gui/left_toe_pos", controllerDataOut.left_toe_pos);
	nh.getParam("/atrias_gui/right_toe_pos", controllerDataOut.right_toe_pos);
	int tauMode;
	nh.getParam("/atrias_gui/tauMode", tauMode);
	controllerDataOut.tauMode = tauMode;
	nh.getParam("/atrias_gui/manualTau", controllerDataOut.manualTau);
	nh.getParam("/atrias_gui/maxCurrent", controllerDataOut.maxCurrent);

	// Configure the GUI
	leg_pos_kp_spinbutton->set_value(controllerDataOut.leg_pos_kp);
	leg_pos_kd_spinbutton->set_value(controllerDataOut.leg_pos_kd);
	hip_pos_kp_spinbutton->set_value(controllerDataOut.hip_pos_kp);
	hip_pos_kd_spinbutton->set_value(controllerDataOut.hip_pos_kd);
	main_controller_combobox->set_active(controllerDataOut.main_controller);
	left_toe_pos_spinbutton->set_value(controllerDataOut.left_toe_pos);
	right_toe_pos_spinbutton->set_value(controllerDataOut.right_toe_pos);
	tau_control_checkbutton->set_active((atrias::controller::TauSource) controllerDataOut.tauMode == atrias::controller::TauSource::GUI);
	tau_hscale->set_value(controllerDataOut.manualTau);
	cur_limit_spinbutton->set_value(controllerDataOut.maxCurrent);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
	nh.setParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
	nh.setParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
	nh.setParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
	nh.setParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
	nh.setParam("/atrias_gui/main_controller", controllerDataOut.main_controller);
	nh.setParam("/atrias_gui/left_toe_pos", controllerDataOut.left_toe_pos);
	nh.setParam("/atrias_gui/right_toe_pos", controllerDataOut.right_toe_pos);
	nh.setParam("/atrias_gui/tauMode", controllerDataOut.tauMode);
	nh.setParam("/atrias_gui/manualTau", controllerDataOut.manualTau);
	nh.setParam("/atrias_gui/maxCurrent", controllerDataOut.maxCurrent);
}

//! \brief Update the GUI.
void guiUpdate() {
	// Set values in controllerDataOut variable here
	controllerDataOut.leg_pos_kp = leg_pos_kp_spinbutton->get_value();
	controllerDataOut.leg_pos_kd = leg_pos_kd_spinbutton->get_value();
	controllerDataOut.hip_pos_kp = hip_pos_kp_spinbutton->get_value();
	controllerDataOut.hip_pos_kd = hip_pos_kd_spinbutton->get_value();
	controllerDataOut.main_controller = (uint8_t)main_controller_combobox->get_active_row_number();
	controllerDataOut.left_toe_pos = left_toe_pos_spinbutton->get_value();
	controllerDataOut.right_toe_pos = right_toe_pos_spinbutton->get_value();
	controllerDataOut.tauMode = (tau_control_checkbutton->get_active()) ?
		(atrias::controller::TauSource_t) atrias::controller::TauSource::GUI :
		(atrias::controller::TauSource_t) atrias::controller::TauSource::STANCE_LEG_ANGLE;
	controllerDataOut.manualTau = tau_hscale->get_value();
	controllerDataOut.maxCurrent = cur_limit_spinbutton->get_value();

	// Publish
	pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}
