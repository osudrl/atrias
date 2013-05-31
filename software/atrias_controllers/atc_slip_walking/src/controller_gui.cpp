/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 */

#include <atc_slip_walking/controller_gui.h>

void flight_td_pressed()  {controllerDataOut.flight_td = 1;}
void flight_td_released() {controllerDataOut.flight_td = 0;}
void flight_to_pressed()  {controllerDataOut.flight_to = 1;}
void flight_to_released() {controllerDataOut.flight_to = 0;}

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	// Get widgets
	gui->get_widget("walking_state_spinbutton", walking_state_spinbutton);
	gui->get_widget("slip_leg_spinbutton", slip_leg_spinbutton);
	gui->get_widget("atrias_spring_spinbutton", atrias_spring_spinbutton);
	gui->get_widget("swing_leg_retraction_spinbutton", swing_leg_retraction_spinbutton);
	gui->get_widget("force_threshold_td_spinbutton", force_threshold_td_spinbutton);
	gui->get_widget("force_threshold_to_spinbutton", force_threshold_to_spinbutton);
	gui->get_widget("position_threshold_td_spinbutton", position_threshold_td_spinbutton);
	gui->get_widget("stance_leg_target_spinbutton", stance_leg_target_spinbutton);
	gui->get_widget("flight_leg_target_spinbutton", flight_leg_target_spinbutton);
	gui->get_widget("leg_pos_kp_spinbutton", leg_pos_kp_spinbutton);
	gui->get_widget("leg_for_kp_spinbutton", leg_for_kp_spinbutton);
	gui->get_widget("leg_for_kd_spinbutton", leg_for_kd_spinbutton);
	gui->get_widget("leg_pos_kd_spinbutton", leg_pos_kd_spinbutton);
	gui->get_widget("hip_pos_kp_spinbutton", hip_pos_kp_spinbutton);
	gui->get_widget("hip_pos_kd_spinbutton", hip_pos_kd_spinbutton);
	gui->get_widget("left_toe_pos_spinbutton", left_toe_pos_spinbutton);
	gui->get_widget("right_toe_pos_spinbutton", right_toe_pos_spinbutton);
	gui->get_widget("main_controller_combobox", main_controller_combobox);
	gui->get_widget("gait_transitions_combobox", gait_transitions_combobox);
	gui->get_widget("flight_td_button", flight_td_button);
	gui->get_widget("flight_to_button", flight_to_button);
	gui->get_widget("apply_hip_torque_togglebutton", apply_hip_torque_togglebutton);
	gui->get_widget("hip_torque_spinbutton", hip_torque_spinbutton);
	gui->get_widget("td_force_spinbutton", td_force_spinbutton);
	gui->get_widget("to_force_spinbutton", to_force_spinbutton);
	gui->get_widget("td_position_spinbutton", td_position_spinbutton);

	// Set ranges
	walking_state_spinbutton->set_range(0, 3);
	slip_leg_spinbutton->set_range(0.75, 0.95);
	atrias_spring_spinbutton->set_range(1000.0, 5000.0);
	swing_leg_retraction_spinbutton->set_range(0.0, 0.15);
	force_threshold_td_spinbutton->set_range(-100.0, 100.0);
	force_threshold_to_spinbutton->set_range(-100.0, 100.0);
	position_threshold_td_spinbutton->set_range(0.0, 0.05);
	stance_leg_target_spinbutton->set_range(M_PI/2.0, M_PI/2.0 + 0.40);
	flight_leg_target_spinbutton->set_range(M_PI/2.0 - 0.40, M_PI/2.0);
	leg_pos_kp_spinbutton->set_range(0.0, 1500.0);
	leg_for_kp_spinbutton->set_range(0.0, 1500.0);
	leg_for_kd_spinbutton->set_range(0.0, 50.0);
	leg_pos_kd_spinbutton->set_range(0.0, 100.0);
	hip_pos_kp_spinbutton->set_range(0.0, 300.0);
	hip_pos_kd_spinbutton->set_range(0.0, 50.0);
	left_toe_pos_spinbutton->set_range(2.1, 2.5);
	right_toe_pos_spinbutton->set_range(2.1, 2.5);
	td_force_spinbutton->set_range(-100.0, 100.0);
	to_force_spinbutton->set_range(-100.0, 100.0);
	td_position_spinbutton->set_range(-0.05, 0.05);
	hip_torque_spinbutton->set_range(0.0, 15.0);


	// Set increments
	walking_state_spinbutton->set_increments(1, 0);
	slip_leg_spinbutton->set_increments(0.01, 0.0);
	atrias_spring_spinbutton->set_increments(10.0, 0.0);
	swing_leg_retraction_spinbutton->set_increments(0.01, 0.0);
	force_threshold_td_spinbutton->set_increments(1.0, 0.0);
	force_threshold_to_spinbutton->set_increments(1.0, 0.0);
	position_threshold_td_spinbutton->set_increments(0.001, 0.0);
	stance_leg_target_spinbutton->set_increments(0.01, 0.0);
	flight_leg_target_spinbutton->set_increments(0.01, 0.0);
	leg_pos_kp_spinbutton->set_increments(10.0, 0.0);
	leg_for_kp_spinbutton->set_increments(10.0, 0.0);
	leg_for_kd_spinbutton->set_increments(1.0, 0.0);
	leg_pos_kd_spinbutton->set_increments(1.0, 0.0);
	hip_pos_kp_spinbutton->set_increments(10.0, 0.0);
	hip_pos_kd_spinbutton->set_increments(1.0, 0.0);
	left_toe_pos_spinbutton->set_increments(0.01, 0.0);
	right_toe_pos_spinbutton->set_increments(0.01, 0.0);
	td_force_spinbutton->set_increments(0.1, 0.0);
	to_force_spinbutton->set_increments(0.1, 0.0);
	td_position_spinbutton->set_increments(0.001, 0.0);
	hip_torque_spinbutton->set_increments(0.5, 0.0);

	// Set values
	walking_state_spinbutton->set_value(0);
	slip_leg_spinbutton->set_value(0.93);
	atrias_spring_spinbutton->set_value(1400.0);
	swing_leg_retraction_spinbutton->set_value(0.12);
	force_threshold_td_spinbutton->set_value(50.0);
	force_threshold_to_spinbutton->set_value(30.0);
	position_threshold_td_spinbutton->set_value(0.02);
	stance_leg_target_spinbutton->set_value(1.6);
	flight_leg_target_spinbutton->set_value(1.3);
	leg_pos_kp_spinbutton->set_value(200.0);
	leg_pos_kd_spinbutton->set_value(10.0);
	leg_for_kp_spinbutton->set_value(100.0);
	leg_for_kd_spinbutton->set_value(2.0);
	hip_pos_kp_spinbutton->set_value(150.0);
	hip_pos_kd_spinbutton->set_value(10.0);
	left_toe_pos_spinbutton->set_value(2.15);
	right_toe_pos_spinbutton->set_value(2.45);
	hip_torque_spinbutton->set_value(7.0);
	
	// Connect buttons to functions
	flight_td_button->signal_pressed().connect(sigc::ptr_fun((void(*)())flight_td_pressed));
	flight_td_button->signal_released().connect(sigc::ptr_fun((void(*)())flight_td_released));
	flight_to_button->signal_pressed().connect(sigc::ptr_fun((void(*)())flight_to_pressed));
	flight_to_button->signal_released().connect(sigc::ptr_fun((void(*)())flight_to_released));

	// Set up subscriber and publisher.
	sub = nh.subscribe("ATCSlipWalking_status", 0, controllerCallback);
	pub = nh.advertise<atc_slip_walking::controller_input>("ATCSlipWalking_input", 0);
	return true;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_slip_walking::controller_status &status) {
	controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
	// Get parameters in the atrias_gui namespace
	nh.getParam("/atrias_gui/walking_state", controllerDataOut.walking_state);
	nh.getParam("/atrias_gui/slip_leg", controllerDataOut.slip_leg);
	nh.getParam("/atrias_gui/atrias_spring", controllerDataOut.atrias_spring);
	nh.getParam("/atrias_gui/swing_leg_retraction", controllerDataOut.swing_leg_retraction);
	nh.getParam("/atrias_gui/force_threshold_td", controllerDataOut.force_threshold_td);
	nh.getParam("/atrias_gui/force_threshold_to", controllerDataOut.force_threshold_to);
	nh.getParam("/atrias_gui/position_threshold_td", controllerDataOut.position_threshold_td);
	nh.getParam("/atrias_gui/stance_leg_target", controllerDataOut.stance_leg_target);
	nh.getParam("/atrias_gui/flight_leg_target", controllerDataOut.flight_leg_target);
	nh.getParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
	nh.getParam("/atrias_gui/leg_for_kp", controllerDataOut.leg_for_kp);
	nh.getParam("/atrias_gui/leg_for_kd", controllerDataOut.leg_for_kd);
	nh.getParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
	nh.getParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
	nh.getParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
	int main_controller;
	nh.getParam("/atrias_gui/main_controller", main_controller);
	controllerDataOut.main_controller = (uint8_t)main_controller;
	int gait_transitions;
	nh.getParam("/atrias_gui/gait_transitions", gait_transitions);
	controllerDataOut.gait_transitions = (uint8_t)gait_transitions;
	nh.getParam("/atrias_gui/td_force", controllerDataOut.td_force);
	nh.getParam("/atrias_gui/to_force", controllerDataOut.to_force);
	nh.getParam("/atrias_gui/td_position", controllerDataOut.td_position);
	nh.getParam("/atrias_gui/left_toe_pos", controllerDataOut.left_toe_pos);
	nh.getParam("/atrias_gui/right_toe_pos", controllerDataOut.right_toe_pos);
	nh.getParam("/atrias_gui/hip_torque", controllerDataOut.hip_torque);

	// Configure the GUI
	walking_state_spinbutton->set_value(controllerDataOut.walking_state);
	slip_leg_spinbutton->set_value(controllerDataOut.slip_leg);
	atrias_spring_spinbutton->set_value(controllerDataOut.atrias_spring);
	force_threshold_td_spinbutton->set_value(controllerDataOut.force_threshold_td);
	force_threshold_to_spinbutton->set_value(controllerDataOut.force_threshold_to);
	position_threshold_td_spinbutton->set_value(controllerDataOut.position_threshold_td);
	leg_pos_kp_spinbutton->set_value(controllerDataOut.leg_pos_kp);
	leg_for_kp_spinbutton->set_value(controllerDataOut.leg_for_kp);
	leg_for_kd_spinbutton->set_value(controllerDataOut.leg_for_kd);
	leg_pos_kd_spinbutton->set_value(controllerDataOut.leg_pos_kd);
	hip_pos_kp_spinbutton->set_value(controllerDataOut.hip_pos_kp);
	hip_pos_kd_spinbutton->set_value(controllerDataOut.hip_pos_kd);
	main_controller_combobox->set_active(controllerDataOut.main_controller);
	gait_transitions_combobox->set_active(controllerDataOut.gait_transitions);
	td_force_spinbutton->set_value(controllerDataOut.td_force);
	to_force_spinbutton->set_value(controllerDataOut.to_force);
	td_position_spinbutton->set_value(controllerDataOut.td_position);
	left_toe_pos_spinbutton->set_value(controllerDataOut.left_toe_pos);
	right_toe_pos_spinbutton->set_value(controllerDataOut.right_toe_pos);
	hip_torque_spinbutton->set_value(controllerDataOut.hip_torque);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
	nh.setParam("/atrias_gui/walking_state", controllerDataOut.walking_state);
	nh.setParam("/atrias_gui/slip_leg", controllerDataOut.slip_leg);
	nh.setParam("/atrias_gui/atrias_spring", controllerDataOut.atrias_spring);
	nh.setParam("/atrias_gui/swing_leg_retraction", controllerDataOut.swing_leg_retraction);
	nh.setParam("/atrias_gui/force_threshold_td", controllerDataOut.force_threshold_td);
	nh.setParam("/atrias_gui/force_threshold_to", controllerDataOut.force_threshold_to);
	nh.setParam("/atrias_gui/position_threshold_td", controllerDataOut.position_threshold_td);
	nh.setParam("/atrias_gui/stance_leg_target", controllerDataOut.stance_leg_target);
	nh.setParam("/atrias_gui/flight_leg_target", controllerDataOut.flight_leg_target);
	nh.setParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
	nh.setParam("/atrias_gui/leg_for_kp", controllerDataOut.leg_for_kp);
	nh.setParam("/atrias_gui/leg_for_kd", controllerDataOut.leg_for_kd);
	nh.setParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
	nh.setParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
	nh.setParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
	nh.setParam("/atrias_gui/main_controller", controllerDataOut.main_controller);
	nh.setParam("/atrias_gui/gait_transitions", controllerDataOut.gait_transitions);
	nh.setParam("/atrias_gui/td_force", controllerDataOut.td_force);
	nh.setParam("/atrias_gui/to_force", controllerDataOut.to_force);
	nh.setParam("/atrias_gui/td_position", controllerDataOut.td_position);
	nh.setParam("/atrias_gui/left_toe_pos", controllerDataOut.left_toe_pos);
	nh.setParam("/atrias_gui/right_toe_pos", controllerDataOut.right_toe_pos);
	nh.setParam("/atrias_gui/hip_torque", controllerDataOut.hip_torque);
}

//! \brief Update the GUI.
void guiUpdate() {
	// Set GUI values from controllerDataOut
	walking_state_spinbutton->set_value(controllerDataIn.walking_state);
	td_force_spinbutton->set_value(controllerDataIn.td_force);
	to_force_spinbutton->set_value(controllerDataIn.to_force);
	td_position_spinbutton->set_value(controllerDataIn.td_position);
	
	// Set values in controllerDataOut variable here
	controllerDataOut.slip_leg = slip_leg_spinbutton->get_value();
	controllerDataOut.atrias_spring = atrias_spring_spinbutton->get_value();
	controllerDataOut.swing_leg_retraction = swing_leg_retraction_spinbutton->get_value();
	controllerDataOut.force_threshold_td = force_threshold_td_spinbutton->get_value();
	controllerDataOut.force_threshold_to = force_threshold_to_spinbutton->get_value();
	controllerDataOut.position_threshold_td = position_threshold_td_spinbutton->get_value();
	controllerDataOut.stance_leg_target = stance_leg_target_spinbutton->get_value();
	controllerDataOut.flight_leg_target = flight_leg_target_spinbutton->get_value();
	controllerDataOut.leg_pos_kp = leg_pos_kp_spinbutton->get_value();
	controllerDataOut.leg_for_kp = leg_for_kp_spinbutton->get_value();
	controllerDataOut.leg_for_kd = leg_for_kd_spinbutton->get_value();
	controllerDataOut.leg_pos_kd = leg_pos_kd_spinbutton->get_value();
	controllerDataOut.hip_pos_kp = hip_pos_kp_spinbutton->get_value();
	controllerDataOut.hip_pos_kd = hip_pos_kd_spinbutton->get_value();
	controllerDataOut.main_controller = (uint8_t)main_controller_combobox->get_active_row_number();
	controllerDataOut.gait_transitions = (uint8_t)gait_transitions_combobox->get_active_row_number();
	controllerDataOut.apply_hip_torque = apply_hip_torque_togglebutton->get_active();
	controllerDataOut.left_toe_pos = left_toe_pos_spinbutton->get_value();
	controllerDataOut.right_toe_pos = right_toe_pos_spinbutton->get_value();
	controllerDataOut.hip_torque = hip_torque_spinbutton->get_value();

	// publish the controller input
	pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}
