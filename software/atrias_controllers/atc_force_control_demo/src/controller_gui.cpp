/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 *  \brief Orocos Component code for the atc_force_control_demo controller.
 */

#include <atc_force_control_demo/controller_gui.h>

// guiInit
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {

	// Get widgets
    gui->get_widget("leg_pos_kp_spinbutton", leg_pos_kp_spinbutton);
    gui->get_widget("leg_pos_kd_spinbutton", leg_pos_kd_spinbutton);
    gui->get_widget("leg_for_kp_spinbutton", leg_for_kp_spinbutton);
    gui->get_widget("leg_for_kd_spinbutton", leg_for_kd_spinbutton);
    gui->get_widget("hip_pos_kp_spinbutton", hip_pos_kp_spinbutton);
    gui->get_widget("hip_pos_kd_spinbutton", hip_pos_kd_spinbutton);
    gui->get_widget("left_leg_len_spinbutton", left_leg_len_spinbutton);
    gui->get_widget("right_leg_len_spinbutton", right_leg_len_spinbutton);
    gui->get_widget("left_leg_ang_spinbutton", left_leg_ang_spinbutton);
    gui->get_widget("right_leg_ang_spinbutton", right_leg_ang_spinbutton);
    gui->get_widget("left_fx_spinbutton", left_fx_spinbutton);
    gui->get_widget("right_fx_spinbutton", right_fx_spinbutton);
    gui->get_widget("left_fz_spinbutton", left_fz_spinbutton);
    gui->get_widget("right_fz_spinbutton", right_fz_spinbutton);
    gui->get_widget("left_ampx_spinbutton", left_ampx_spinbutton);
    gui->get_widget("right_ampx_spinbutton", right_ampx_spinbutton);
    gui->get_widget("left_ampz_spinbutton", left_ampz_spinbutton);
    gui->get_widget("right_ampz_spinbutton", right_ampz_spinbutton);
    gui->get_widget("left_freqx_spinbutton", left_freqx_spinbutton);
    gui->get_widget("right_freqx_spinbutton", right_freqx_spinbutton);
    gui->get_widget("left_freqz_spinbutton", left_freqz_spinbutton);
    gui->get_widget("right_freqz_spinbutton", right_freqz_spinbutton);
    gui->get_widget("left_offx_spinbutton", left_offx_spinbutton);
    gui->get_widget("right_offx_spinbutton", right_offx_spinbutton);
    gui->get_widget("left_offz_spinbutton", left_offz_spinbutton);
    gui->get_widget("right_offz_spinbutton", right_offz_spinbutton);
    gui->get_widget("left_controller_combobox", left_controller_combobox);
    gui->get_widget("right_controller_combobox", right_controller_combobox);
    
	// Set ranges
    leg_pos_kp_spinbutton->set_range(0.0, 5000.0);
    leg_pos_kd_spinbutton->set_range(0.0, 100.0);
    leg_for_kp_spinbutton->set_range(0.0, 1500.0);
    leg_for_kd_spinbutton->set_range(0.0, 50.0);
    hip_pos_kp_spinbutton->set_range(0.0, 250.0);
    hip_pos_kd_spinbutton->set_range(0.0, 25.0);
    left_leg_len_spinbutton->set_range(0.45, 0.90);
    right_leg_len_spinbutton->set_range(0.45, 0.90);
    left_leg_ang_spinbutton->set_range(M_PI/4.0, 3.0*M_PI/4.0);
    right_leg_ang_spinbutton->set_range(M_PI/4.0, 3.0*M_PI/4.0);
    left_fx_spinbutton->set_range(-1000.0, 1000.0);
    right_fx_spinbutton->set_range(-1000.0, 1000.0);
    left_fz_spinbutton->set_range(-1000.0, 1000.0);
    right_fz_spinbutton->set_range(-1000.0, 1000.0);
    left_ampx_spinbutton->set_range(0.0, 500.0);
    right_ampx_spinbutton->set_range(0.0, 500.0);
    left_ampz_spinbutton->set_range(0.0, 500.0);
    right_ampz_spinbutton->set_range(0.0, 500.0);
    left_freqx_spinbutton->set_range(0.0, 15.0);
    right_freqx_spinbutton->set_range(0.0, 15.0);
    left_freqz_spinbutton->set_range(0.0, 15.0);
    right_freqz_spinbutton->set_range(0.0, 15.0);
    left_offx_spinbutton->set_range(-500.0, 500.0);
    right_offx_spinbutton->set_range(-500.0, 500.0);
    left_offz_spinbutton->set_range(-500.0, 500.0);
    right_offz_spinbutton->set_range(-500.0, 500.0);
	
	// Set default values
    leg_pos_kp_spinbutton->set_increments(10.0, 0.0);
    leg_pos_kd_spinbutton->set_increments(1.0, 0.0);
    leg_for_kp_spinbutton->set_increments(10.0, 0.0);
    leg_for_kd_spinbutton->set_increments(1.0, 0.0);
    hip_pos_kp_spinbutton->set_increments(10.0, 0.0);
    hip_pos_kd_spinbutton->set_increments(1.0, 0.0);
    left_leg_len_spinbutton->set_increments(0.01, 0.0);
    right_leg_len_spinbutton->set_increments(0.01, 0.0);
    left_leg_ang_spinbutton->set_increments(0.01, 0.0);
    right_leg_ang_spinbutton->set_increments(0.01, 0.0);
    left_fx_spinbutton->set_increments(1.0, 0.0);
    right_fx_spinbutton->set_increments(1.0, 0.0);
    left_fz_spinbutton->set_increments(1.0, 0.0);
    right_fz_spinbutton->set_increments(1.0, 0.0);
    left_ampx_spinbutton->set_increments(1.0, 0.0);
    right_ampx_spinbutton->set_increments(1.0, 0.0);
    left_ampz_spinbutton->set_increments(1.0, 0.0);
    right_ampz_spinbutton->set_increments(1.0, 0.0);
    left_freqx_spinbutton->set_increments(0.1, 0.0);
    right_freqx_spinbutton->set_increments(0.1, 0.0);
    left_freqz_spinbutton->set_increments(0.1, 0.0);
    right_freqz_spinbutton->set_increments(0.1, 0.0);
    left_offx_spinbutton->set_increments(1.0, 0.0);
    right_offx_spinbutton->set_increments(1.0, 0.0);
    left_offz_spinbutton->set_increments(1.0, 0.0);
    right_offz_spinbutton->set_increments(1.0, 0.0);
	
	// Set values
    leg_pos_kp_spinbutton->set_value(500.0);
    leg_pos_kd_spinbutton->set_value(25.0);
    leg_for_kp_spinbutton->set_value(1000.0);
    leg_for_kd_spinbutton->set_value(8.0);
    hip_pos_kp_spinbutton->set_value(150.0);
    hip_pos_kd_spinbutton->set_value(8.0);
    left_leg_len_spinbutton->set_value(0.85);
    right_leg_len_spinbutton->set_value(0.85);
    left_leg_ang_spinbutton->set_value(M_PI/2.0);
    right_leg_ang_spinbutton->set_value(M_PI/2.0);
    left_fx_spinbutton->set_value(0.0);
    right_fx_spinbutton->set_value(0.0);
    left_fz_spinbutton->set_value(0.0);
    right_fz_spinbutton->set_value(0.0);
    left_ampx_spinbutton->set_value(0.0);
    right_ampx_spinbutton->set_value(0.0);
    left_ampz_spinbutton->set_value(0.0);
    right_ampz_spinbutton->set_value(0.0);
    left_freqx_spinbutton->set_value(0.0);
    right_freqx_spinbutton->set_value(0.0);
    left_freqz_spinbutton->set_value(0.0);
    right_freqz_spinbutton->set_value(0.0);
    left_offx_spinbutton->set_value(0.0);
    right_offx_spinbutton->set_value(0.0);
    left_offz_spinbutton->set_value(0.0);
    right_offz_spinbutton->set_value(0.0);

	// Set up subscriber and publisher.
	sub = nh.subscribe("ATCForceControlDemo_status", 0, controllerCallback);
	pub = nh.advertise<atc_force_control_demo::controller_input>("ATCForceControlDemo_input", 0);

	return true;

} // guiInit


// controllerCallback
void controllerCallback(const atc_force_control_demo::controller_status &status) {

    controllerDataIn = status;

} // controllerCallback


// getParameters
void getParameters() {

    // Get parameters in the atrias_gui namespace.
    nh.getParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
    nh.getParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
    nh.getParam("/atrias_gui/leg_for_kp", controllerDataOut.leg_for_kp);
    nh.getParam("/atrias_gui/leg_for_kd", controllerDataOut.leg_for_kd);
    nh.getParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
    nh.getParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
    nh.getParam("/atrias_gui/left_leg_len", controllerDataOut.left_leg_len);
    nh.getParam("/atrias_gui/right_leg_len", controllerDataOut.right_leg_len);
    nh.getParam("/atrias_gui/left_leg_ang", controllerDataOut.left_leg_ang);
    nh.getParam("/atrias_gui/right_leg_ang", controllerDataOut.right_leg_ang);
    nh.getParam("/atrias_gui/left_fx", controllerDataOut.left_fx);
    nh.getParam("/atrias_gui/right_fx", controllerDataOut.right_fx);
    nh.getParam("/atrias_gui/left_fz", controllerDataOut.left_fz);
    nh.getParam("/atrias_gui/right_fz", controllerDataOut.right_fz);
    nh.getParam("/atrias_gui/left_ampx", controllerDataOut.left_ampx);
    nh.getParam("/atrias_gui/right_ampx", controllerDataOut.right_ampx);
    nh.getParam("/atrias_gui/left_ampz", controllerDataOut.left_ampz);
    nh.getParam("/atrias_gui/right_ampz", controllerDataOut.right_ampz);
    nh.getParam("/atrias_gui/left_freqx", controllerDataOut.left_freqx);
    nh.getParam("/atrias_gui/right_freqx", controllerDataOut.right_freqx);
    nh.getParam("/atrias_gui/left_freqz", controllerDataOut.left_freqz);
    nh.getParam("/atrias_gui/right_freqz", controllerDataOut.right_freqz);
    nh.getParam("/atrias_gui/left_offx", controllerDataOut.left_offx);
    nh.getParam("/atrias_gui/right_offx", controllerDataOut.right_offx);
    nh.getParam("/atrias_gui/left_offz", controllerDataOut.left_offz);
    nh.getParam("/atrias_gui/right_offz", controllerDataOut.right_offz);
    int left_controller;
    nh.getParam("/atrias_gui/left_controller", left_controller);
    controllerDataOut.left_controller = (uint8_t)left_controller;
    int right_controller;
    nh.getParam("/atrias_gui/right_controller", right_controller);
    controllerDataOut.right_controller = (uint8_t)right_controller;

    // Configure the GUI.
    leg_pos_kp_spinbutton->set_value(controllerDataOut.leg_pos_kp);
    leg_pos_kd_spinbutton->set_value(controllerDataOut.leg_pos_kd);
    leg_for_kp_spinbutton->set_value(controllerDataOut.leg_for_kp);
    leg_for_kd_spinbutton->set_value(controllerDataOut.leg_for_kd);
    hip_pos_kp_spinbutton->set_value(controllerDataOut.hip_pos_kp);
    hip_pos_kd_spinbutton->set_value(controllerDataOut.hip_pos_kd);
    left_leg_len_spinbutton->set_value(controllerDataOut.left_leg_len);
    right_leg_len_spinbutton->set_value(controllerDataOut.right_leg_len);
    left_leg_ang_spinbutton->set_value(controllerDataOut.left_leg_ang);
    right_leg_ang_spinbutton->set_value(controllerDataOut.right_leg_ang);
    left_fx_spinbutton->set_value(controllerDataOut.left_fx);
    right_fx_spinbutton->set_value(controllerDataOut.right_fx);
    left_fz_spinbutton->set_value(controllerDataOut.left_fz);
    right_fz_spinbutton->set_value(controllerDataOut.right_fz);
    left_ampx_spinbutton->set_value(controllerDataOut.left_ampx);
    right_ampx_spinbutton->set_value(controllerDataOut.right_ampx);
    left_ampz_spinbutton->set_value(controllerDataOut.left_ampz);
    right_ampz_spinbutton->set_value(controllerDataOut.right_ampz);
    left_freqx_spinbutton->set_value(controllerDataOut.left_freqx);
    right_freqx_spinbutton->set_value(controllerDataOut.right_freqx);
    left_freqz_spinbutton->set_value(controllerDataOut.left_freqz);
    right_freqz_spinbutton->set_value(controllerDataOut.right_freqz);
    left_offx_spinbutton->set_value(controllerDataOut.left_offx);
    right_offx_spinbutton->set_value(controllerDataOut.right_offx);
    left_offz_spinbutton->set_value(controllerDataOut.left_offz);
    right_offz_spinbutton->set_value(controllerDataOut.right_offz); 
    left_controller_combobox->set_active(controllerDataOut.left_controller);
    right_controller_combobox->set_active(controllerDataOut.right_controller);  

} // getParameters


// setParameters
void setParameters() {

	// Set parameters in the atrias_gui namespace.
    nh.setParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
    nh.setParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
    nh.setParam("/atrias_gui/leg_for_kp", controllerDataOut.leg_for_kp);
    nh.setParam("/atrias_gui/leg_for_kd", controllerDataOut.leg_for_kd);
    nh.setParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
    nh.setParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
    nh.setParam("/atrias_gui/left_leg_len", controllerDataOut.left_leg_len);
    nh.setParam("/atrias_gui/right_leg_len", controllerDataOut.right_leg_len);
    nh.setParam("/atrias_gui/left_leg_ang", controllerDataOut.left_leg_ang);
    nh.setParam("/atrias_gui/right_leg_ang", controllerDataOut.right_leg_ang);
    nh.setParam("/atrias_gui/left_fx", controllerDataOut.left_fx);
    nh.setParam("/atrias_gui/right_fx", controllerDataOut.right_fx);
    nh.setParam("/atrias_gui/left_fz", controllerDataOut.left_fz);
    nh.setParam("/atrias_gui/right_fz", controllerDataOut.right_fz);
    nh.setParam("/atrias_gui/left_ampx", controllerDataOut.left_ampx);
    nh.setParam("/atrias_gui/right_ampx", controllerDataOut.right_ampx);
    nh.setParam("/atrias_gui/left_ampz", controllerDataOut.left_ampz);
    nh.setParam("/atrias_gui/right_ampz", controllerDataOut.right_ampz);
    nh.setParam("/atrias_gui/left_freqx", controllerDataOut.left_freqx);
    nh.setParam("/atrias_gui/right_freqx", controllerDataOut.right_freqx);
    nh.setParam("/atrias_gui/left_freqz", controllerDataOut.left_freqz);
    nh.setParam("/atrias_gui/right_freqz", controllerDataOut.right_freqz);
    nh.setParam("/atrias_gui/left_offx", controllerDataOut.left_offx);
    nh.setParam("/atrias_gui/right_offx", controllerDataOut.right_offx);
    nh.setParam("/atrias_gui/left_offz", controllerDataOut.left_offz);
    nh.setParam("/atrias_gui/right_offz", controllerDataOut.right_offz);
    nh.setParam("/atrias_gui/left_controller", controllerDataOut.left_controller);
    nh.setParam("/atrias_gui/right_controller", controllerDataOut.right_controller);

} // setParameters


// guiUpdate
void guiUpdate() {

    controllerDataOut.leg_pos_kp = leg_pos_kp_spinbutton->get_value();
    controllerDataOut.leg_pos_kd = leg_pos_kd_spinbutton->get_value();
    controllerDataOut.leg_for_kp = leg_for_kp_spinbutton->get_value();
    controllerDataOut.leg_for_kd = leg_for_kd_spinbutton->get_value();
    controllerDataOut.hip_pos_kp = hip_pos_kp_spinbutton->get_value();
    controllerDataOut.hip_pos_kd = hip_pos_kd_spinbutton->get_value();
    controllerDataOut.left_leg_len = left_leg_len_spinbutton->get_value();
    controllerDataOut.right_leg_len = right_leg_len_spinbutton->get_value();
    controllerDataOut.left_leg_ang = left_leg_ang_spinbutton->get_value();
    controllerDataOut.right_leg_ang = right_leg_ang_spinbutton->get_value();
    controllerDataOut.left_fx = left_fx_spinbutton->get_value();
    controllerDataOut.right_fx = right_fx_spinbutton->get_value();
    controllerDataOut.left_fz = left_fz_spinbutton->get_value();
    controllerDataOut.right_fz = right_fz_spinbutton->get_value();
    controllerDataOut.left_ampx = left_ampx_spinbutton->get_value();
    controllerDataOut.right_ampx = right_ampx_spinbutton->get_value();
    controllerDataOut.left_ampz = left_ampz_spinbutton->get_value();
    controllerDataOut.right_ampz = right_ampz_spinbutton->get_value();
    controllerDataOut.left_freqx = left_freqx_spinbutton->get_value();
    controllerDataOut.right_freqx = right_freqx_spinbutton->get_value();
    controllerDataOut.left_freqz = left_freqz_spinbutton->get_value();
    controllerDataOut.right_freqz = right_freqz_spinbutton->get_value();
    controllerDataOut.left_offx = left_offx_spinbutton->get_value();
    controllerDataOut.right_offx = right_offx_spinbutton->get_value();
    controllerDataOut.left_offz = left_offz_spinbutton->get_value();
    controllerDataOut.right_offz = right_offz_spinbutton->get_value();
    controllerDataOut.left_controller = (uint8_t)left_controller_combobox->get_active_row_number();
    controllerDataOut.right_controller = (uint8_t)right_controller_combobox->get_active_row_number();

    pub.publish(controllerDataOut);

} // guiUpdate


// guiTakedown
void guiTakedown() {
	// Stuff
} // guiTakedown

