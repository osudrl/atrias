/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 */

#include <atc_heuristic_slip_walking/controller_gui.h>

void manual_event_pressed()  {controllerDataOut.manual_event = 1;}
void manual_event_released() {controllerDataOut.manual_event = 0;}

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	// Get widgets
	gui->get_widget("walking_state_spinbutton", walking_state_spinbutton);
	gui->get_widget("standing_leg_spinbutton", standing_leg_spinbutton);
	gui->get_widget("slip_leg_spinbutton", slip_leg_spinbutton);
	gui->get_widget("leg_pos_kp_spinbutton", leg_pos_kp_spinbutton);
	gui->get_widget("leg_for_kp_spinbutton", leg_for_kp_spinbutton);
	gui->get_widget("leg_for_kd_spinbutton", leg_for_kd_spinbutton);
	gui->get_widget("leg_pos_kd_spinbutton", leg_pos_kd_spinbutton);
	gui->get_widget("hip_pos_kp_spinbutton", hip_pos_kp_spinbutton);
	gui->get_widget("hip_pos_kd_spinbutton", hip_pos_kd_spinbutton);
	gui->get_widget("main_controller_combobox", main_controller_combobox);
	gui->get_widget("state_trigger_combobox", state_trigger_combobox);
	gui->get_widget("manual_event_button", manual_event_button);

	// Set ranges
	walking_state_spinbutton->set_range(0, 3);
	standing_leg_spinbutton->set_range(0.5, 0.95);
	slip_leg_spinbutton->set_range(0.5, 0.95);
	leg_pos_kp_spinbutton->set_range(0.0, 7500.0);
	leg_for_kp_spinbutton->set_range(0.0, 7500.0);
	leg_for_kd_spinbutton->set_range(0.0, 500.0);
	leg_pos_kd_spinbutton->set_range(0.0, 500.0);
	hip_pos_kp_spinbutton->set_range(0.0, 500.0);
	hip_pos_kd_spinbutton->set_range(0.0, 50.0);

	// Set increments
	walking_state_spinbutton->set_increments(1, 0.0);
	standing_leg_spinbutton->set_increments(0.01, 0.0);
	slip_leg_spinbutton->set_increments(0.01, 0.0);
	leg_pos_kp_spinbutton->set_increments(10.0, 0.0);
	leg_for_kp_spinbutton->set_increments(10.0, 0.0);
	leg_for_kd_spinbutton->set_increments(1.0, 0.0);
	leg_pos_kd_spinbutton->set_increments(1.0, 0.0);
	hip_pos_kp_spinbutton->set_increments(10.0, 0.0);
	hip_pos_kd_spinbutton->set_increments(1.0, 0.0);

	// Set values
	walking_state_spinbutton->set_value(0);
	standing_leg_spinbutton->set_value(0.85);
	slip_leg_spinbutton->set_value(0.80);
	leg_pos_kp_spinbutton->set_value(1000.0);
	leg_for_kp_spinbutton->set_value(1000.0);
	leg_for_kd_spinbutton->set_value(10.0);
	leg_pos_kd_spinbutton->set_value(50.0);
	hip_pos_kp_spinbutton->set_value(200.0);
	hip_pos_kd_spinbutton->set_value(10.0);
	
	// Connect buttons to functions
	manual_event_button->signal_pressed().connect(sigc::ptr_fun((void(*)())manual_event_pressed));
	manual_event_button->signal_released().connect(sigc::ptr_fun((void(*)())manual_event_released));

	// Set up subscriber and publisher.
	sub = nh.subscribe("ATCHeuristicSlipWalking_status", 0, controllerCallback);
	pub = nh.advertise<atc_heuristic_slip_walking::controller_input>("ATCHeuristicSlipWalking_input", 0);
	return true;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_heuristic_slip_walking::controller_status &status) {
	controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
	// Get parameters in the atrias_gui namespace
	nh.getParam("/atrias_gui/walking_state", controllerDataOut.walking_state);
	nh.getParam("/atrias_gui/standing_leg", controllerDataOut.standing_leg);
	nh.getParam("/atrias_gui/slip_leg", controllerDataOut.slip_leg);
	nh.getParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
	nh.getParam("/atrias_gui/leg_for_kp", controllerDataOut.leg_for_kp);
	nh.getParam("/atrias_gui/leg_for_kd", controllerDataOut.leg_for_kd);
	nh.getParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
	nh.getParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
	nh.getParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
	int main_controller;
	nh.getParam("/atrias_gui/main_controller", main_controller);
	controllerDataOut.main_controller = (uint8_t)main_controller;
	int state_trigger;
	nh.getParam("/atrias_gui/state_trigger", state_trigger);
	controllerDataOut.state_trigger = (uint8_t)state_trigger;

	// Configure the GUI
	walking_state_spinbutton->set_value(controllerDataOut.walking_state);
	standing_leg_spinbutton->set_value(controllerDataOut.standing_leg);
	slip_leg_spinbutton->set_value(controllerDataOut.slip_leg);
	leg_pos_kp_spinbutton->set_value(controllerDataOut.leg_pos_kp);
	leg_for_kp_spinbutton->set_value(controllerDataOut.leg_for_kp);
	leg_for_kd_spinbutton->set_value(controllerDataOut.leg_for_kd);
	leg_pos_kd_spinbutton->set_value(controllerDataOut.leg_pos_kd);
	hip_pos_kp_spinbutton->set_value(controllerDataOut.hip_pos_kp);
	hip_pos_kd_spinbutton->set_value(controllerDataOut.hip_pos_kd);
	main_controller_combobox->set_active(controllerDataOut.main_controller);
	state_trigger_combobox->set_active(controllerDataOut.state_trigger);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
	nh.setParam("/atrias_gui/walking_state", controllerDataOut.walking_state);
	nh.setParam("/atrias_gui/standing_leg", controllerDataOut.standing_leg);
	nh.setParam("/atrias_gui/slip_leg", controllerDataOut.slip_leg);
	nh.setParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
	nh.setParam("/atrias_gui/leg_for_kp", controllerDataOut.leg_for_kp);
	nh.setParam("/atrias_gui/leg_for_kd", controllerDataOut.leg_for_kd);
	nh.setParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
	nh.setParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
	nh.setParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
	nh.setParam("/atrias_gui/main_controller", controllerDataOut.main_controller);
	nh.setParam("/atrias_gui/state_trigger", controllerDataOut.state_trigger);
}

//! \brief Update the GUI.
void guiUpdate() {
	// Set values in controllerDataOut variable here
	if ((uint8_t)state_trigger_combobox->get_active_row_number() == 1) {
		walking_state_spinbutton->set_value(controllerDataIn.walking_state);
	}
    
	controllerDataOut.walking_state = walking_state_spinbutton->get_value();
	controllerDataOut.standing_leg = standing_leg_spinbutton->get_value();
	controllerDataOut.slip_leg = slip_leg_spinbutton->get_value();
	controllerDataOut.leg_pos_kp = leg_pos_kp_spinbutton->get_value();
	controllerDataOut.leg_for_kp = leg_for_kp_spinbutton->get_value();
	controllerDataOut.leg_for_kd = leg_for_kd_spinbutton->get_value();
	controllerDataOut.leg_pos_kd = leg_pos_kd_spinbutton->get_value();
	controllerDataOut.hip_pos_kp = hip_pos_kp_spinbutton->get_value();
	controllerDataOut.hip_pos_kd = hip_pos_kd_spinbutton->get_value();
	controllerDataOut.main_controller = (uint8_t)main_controller_combobox->get_active_row_number();
	controllerDataOut.state_trigger = (uint8_t)state_trigger_combobox->get_active_row_number();

	// publish the controller input
	pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}
