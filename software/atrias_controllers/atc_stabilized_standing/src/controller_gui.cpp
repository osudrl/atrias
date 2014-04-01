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

	// Leg gains
	gui->get_widget("leg_pos_kp_spinbutton", leg_pos_kp_spinbutton);
	leg_pos_kp_spinbutton->set_range(0.0, 1000.0);
	leg_pos_kp_spinbutton->set_increments(10.0, 0.0);
	leg_pos_kp_spinbutton->set_value(500.0);
	gui->get_widget("leg_pos_kd_spinbutton", leg_pos_kd_spinbutton);
	leg_pos_kd_spinbutton->set_range(0.0, 100.0);
	leg_pos_kd_spinbutton->set_increments(1.0, 0.0);
	leg_pos_kd_spinbutton->set_value(50.0);

	// Hip gains
	gui->get_widget("hip_pos_kp_spinbutton", hip_pos_kp_spinbutton);
	hip_pos_kp_spinbutton->set_range(0.0, 300.0);
	hip_pos_kp_spinbutton->set_increments(10.0, 0.0);
	hip_pos_kp_spinbutton->set_value(150.0);
	gui->get_widget("hip_pos_kd_spinbutton", hip_pos_kd_spinbutton);
	hip_pos_kd_spinbutton->set_range(0.0, 50.0);
	hip_pos_kd_spinbutton->set_increments(1.0, 0.0);
	hip_pos_kd_spinbutton->set_value(10.0);
	gui->get_widget("left_toe_pos_spinbutton", left_toe_pos_spinbutton);
	left_toe_pos_spinbutton->set_range(2.1, 2.5);
	left_toe_pos_spinbutton->set_increments(0.01, 0.0);
	left_toe_pos_spinbutton->set_value(2.17);
	gui->get_widget("right_toe_pos_spinbutton", right_toe_pos_spinbutton);
	right_toe_pos_spinbutton->set_range(2.1, 2.55);
	right_toe_pos_spinbutton->set_increments(0.01, 0.0);
	right_toe_pos_spinbutton->set_value(2.5);

	// Safeties
	gui->get_widget("current_limit_spinbutton", current_limit_spinbutton);
	current_limit_spinbutton->set_range(0.0, 60.0);
	current_limit_spinbutton->set_increments(5.0, 0.0);
	current_limit_spinbutton->set_value(10.0);
	gui->get_widget("deflection_limit_spinbutton", deflection_limit_spinbutton);
	deflection_limit_spinbutton->set_range(0.0, 0.3);
	deflection_limit_spinbutton->set_increments(0.05, 0.0);
	deflection_limit_spinbutton->set_value(0.25);
	gui->get_widget("velocity_limit_spinbutton", velocity_limit_spinbutton);
	velocity_limit_spinbutton->set_range(0.0, 15.0);
	velocity_limit_spinbutton->set_increments(0.5, 0.0);
	velocity_limit_spinbutton->set_value(6.0);

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

	// Leg gains
	nh.getParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
	leg_pos_kp_spinbutton->set_value(controllerDataOut.leg_pos_kp);
	nh.getParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
	leg_pos_kd_spinbutton->set_value(controllerDataOut.leg_pos_kd);

	// Hip gains
	nh.getParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
	hip_pos_kp_spinbutton->set_value(controllerDataOut.hip_pos_kp);
	nh.getParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
	hip_pos_kd_spinbutton->set_value(controllerDataOut.hip_pos_kd);
	nh.getParam("/atrias_gui/left_toe_pos", controllerDataOut.left_toe_pos);
	left_toe_pos_spinbutton->set_value(controllerDataOut.left_toe_pos);
	nh.getParam("/atrias_gui/right_toe_pos", controllerDataOut.right_toe_pos);
	right_toe_pos_spinbutton->set_value(controllerDataOut.right_toe_pos);

	// Safeties
	nh.getParam("/atrias_gui/current_limit", controllerDataOut.current_limit);
	current_limit_spinbutton->set_value(controllerDataOut.current_limit);
	nh.getParam("/atrias_gui/deflection_limit", controllerDataOut.deflection_limit);
	deflection_limit_spinbutton->set_value(controllerDataOut.deflection_limit);
	nh.getParam("/atrias_gui/velocity_limit", controllerDataOut.velocity_limit);
	velocity_limit_spinbutton->set_value(controllerDataOut.velocity_limit);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
	// Main controller options
	nh.setParam("/atrias_gui/main_controller", controllerDataOut.main_controller);

	// Leg gains
	nh.setParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
	nh.setParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);

	// Hip gains
	nh.setParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
	nh.setParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
	nh.setParam("/atrias_gui/left_toe_pos", controllerDataOut.left_toe_pos);
	nh.setParam("/atrias_gui/right_toe_pos", controllerDataOut.right_toe_pos);

	// Safeties
	nh.setParam("/atrias_gui/current_limit", controllerDataOut.current_limit);
	nh.setParam("/atrias_gui/deflection_limit", controllerDataOut.deflection_limit);
	nh.setParam("/atrias_gui/velocity_limit", controllerDataOut.velocity_limit);
}

//! \brief Update the GUI.
void guiUpdate() {
	// Main controller options
	controllerDataOut.main_controller = (uint8_t)main_controller_combobox->get_active_row_number();

	// Leg gains
	controllerDataOut.leg_pos_kp = leg_pos_kp_spinbutton->get_value();
	controllerDataOut.leg_pos_kd = leg_pos_kd_spinbutton->get_value();

	// Hip gains
	controllerDataOut.hip_pos_kp = hip_pos_kp_spinbutton->get_value();
	controllerDataOut.hip_pos_kd = hip_pos_kd_spinbutton->get_value();
	controllerDataOut.left_toe_pos = left_toe_pos_spinbutton->get_value();
	controllerDataOut.right_toe_pos = right_toe_pos_spinbutton->get_value();

	// Safeties
	controllerDataOut.current_limit = current_limit_spinbutton->get_value();
	controllerDataOut.deflection_limit = deflection_limit_spinbutton->get_value();
	controllerDataOut.velocity_limit = velocity_limit_spinbutton->get_value();

	// publish the controller input
	pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}
