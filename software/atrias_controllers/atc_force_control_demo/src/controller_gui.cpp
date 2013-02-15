/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 *  \brief Orocos Component code for the atc_force_control_demo controller.
 */

#include <atc_force_control_demo/controller_gui.h>

// guiInit =====================================================================
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {

	// Get widgets
    gui->get_widget("leg_pos_kp_spinbutton", leg_pos_kp_spinbutton);
    gui->get_widget("leg_pos_kd_spinbutton", leg_pos_kd_spinbutton);
    gui->get_widget("leg_for_kp_spinbutton", leg_for_kp_spinbutton);
    gui->get_widget("leg_for_kd_spinbutton", leg_for_kd_spinbutton);
    gui->get_widget("hip_pos_kp_spinbutton", hip_pos_kp_spinbutton);
    gui->get_widget("hip_pos_kd_spinbutton", hip_pos_kd_spinbutton);
    gui->get_widget("robot_ks_spinbutton", robot_ks_spinbutton);
    gui->get_widget("robot_kt_spinbutton", robot_kt_spinbutton);
    gui->get_widget("robot_kg_spinbutton", robot_kg_spinbutton);
    gui->get_widget("left_hip_ang_spinbutton", left_hip_ang_spinbutton);
    gui->get_widget("right_hip_ang_spinbutton", right_hip_ang_spinbutton);
    gui->get_widget("left_toe_pos_spinbutton", left_toe_pos_spinbutton);
    gui->get_widget("right_toe_pos_spinbutton", right_toe_pos_spinbutton);
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
    gui->get_widget("constant_hip_radiobutton", constant_hip_radiobutton);
    gui->get_widget("constant_toe_radiobutton", constant_toe_radiobutton);
    gui->get_widget("left_leg_pos_radiobutton", left_leg_pos_radiobutton);
    gui->get_widget("right_leg_pos_radiobutton", right_leg_pos_radiobutton);
    gui->get_widget("left_leg_for_radiobutton", left_leg_for_radiobutton);
    gui->get_widget("right_leg_for_radiobutton", right_leg_for_radiobutton);
    gui->get_widget("left_leg_wave_for_radiobutton", left_leg_wave_for_radiobutton);
    gui->get_widget("right_leg_wave_for_radiobutton", right_leg_wave_for_radiobutton);
    
	// Set ranges.
    leg_pos_kp_spinbutton->set_range(0.0, 7500.0);
    leg_pos_kd_spinbutton->set_range(0.0, 150.0);
    leg_for_kp_spinbutton->set_range(0.0, 7500.0);
    leg_for_kd_spinbutton->set_range(0.0, 150.0);
    hip_pos_kp_spinbutton->set_range(0.0, 250.0);
    hip_pos_kd_spinbutton->set_range(0.0, 25.0);
    robot_ks_spinbutton->set_range(0.0, 10000.0);
    robot_kt_spinbutton->set_range(0.0, 1.0);
    robot_kg_spinbutton->set_range(50.0, 50.0);
    left_hip_ang_spinbutton->set_range(3.0*M_PI/2.0 - 0.3, 3.0*M_PI/2.0 + 0.3);
    right_hip_ang_spinbutton->set_range(3.0*M_PI/2.0 - 0.3, 3.0*M_PI/2.0 + 0.3);
    left_toe_pos_spinbutton->set_range(2.0, 2.3);
    right_toe_pos_spinbutton->set_range(2.3, 2.6);
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
    leg_pos_kp_spinbutton->set_value(500.0);
    leg_pos_kd_spinbutton->set_value(25.0);
    leg_for_kp_spinbutton->set_value(5000.0);
    leg_for_kd_spinbutton->set_value(25.0);
    hip_pos_kp_spinbutton->set_value(100.0);
    hip_pos_kd_spinbutton->set_value(5.0);
    robot_ks_spinbutton->set_value(4118.0);
    robot_kt_spinbutton->set_value(0.0987);
    robot_kg_spinbutton->set_value(50.0);
    left_hip_ang_spinbutton->set_value(3.0*M_PI/2.0);
    right_hip_ang_spinbutton->set_value(3.0*M_PI/2.0);
    left_toe_pos_spinbutton->set_value(2.15);
    right_toe_pos_spinbutton->set_value(2.45);
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
	sub = nh.subscribe("atc_force_control_demo_status", 0, controllerCallback);
	pub = nh.advertise<atc_force_control_demo::controller_input>("atc_force_control_demo_input", 0);

	return true;

} // guiInit

// controllerCallback ==========================================================
void controllerCallback(const atc_force_control_demo::controller_status &status) {
    controllerDataIn = status;

} // controllerCallback

// getParameters ===============================================================
void getParameters() {
    // Get parameters in the atrias_gui namespace.
    nh.getParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
    nh.getParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
    nh.getParam("/atrias_gui/leg_for_kp", controllerDataOut.leg_for_kp);
    nh.getParam("/atrias_gui/leg_for_kd", controllerDataOut.leg_for_kd);
    nh.getParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
    nh.getParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
    nh.getParam("/atrias_gui/robot_ks", controllerDataOut.robot_ks);
    nh.getParam("/atrias_gui/robot_kt", controllerDataOut.robot_kt);
    nh.getParam("/atrias_gui/robot_kg", controllerDataOut.robot_kg);
    nh.getParam("/atrias_gui/left_hip_ang", controllerDataOut.left_hip_ang);
    nh.getParam("/atrias_gui/right_hip_ang", controllerDataOut.right_hip_ang);
    nh.getParam("/atrias_gui/left_toe_pos", controllerDataOut.left_toe_pos);
    nh.getParam("/atrias_gui/right_toe_pos", controllerDataOut.right_toe_pos);
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

    // Configure the GUI.
    leg_pos_kp_spinbutton->set_value(controllerDataOut.leg_pos_kp);
    leg_pos_kd_spinbutton->set_value(controllerDataOut.leg_pos_kd);
    leg_for_kp_spinbutton->set_value(controllerDataOut.leg_for_kp);
    leg_for_kd_spinbutton->set_value(controllerDataOut.leg_for_kd);
    hip_pos_kp_spinbutton->set_value(controllerDataOut.hip_pos_kp);
    hip_pos_kd_spinbutton->set_value(controllerDataOut.hip_pos_kd);
    robot_ks_spinbutton->set_value(controllerDataOut.robot_ks);
    robot_kt_spinbutton->set_value(controllerDataOut.robot_kt);
    robot_kg_spinbutton->set_value(controllerDataOut.robot_kg);
    left_hip_ang_spinbutton->set_value(controllerDataOut.left_hip_ang);
    right_hip_ang_spinbutton->set_value(controllerDataOut.right_hip_ang);
    left_toe_pos_spinbutton->set_value(controllerDataOut.left_toe_pos);
    right_toe_pos_spinbutton->set_value(controllerDataOut.right_toe_pos);
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
    
    

} // getParameters

// setParameters ===============================================================
void setParameters() {
	// Set parameters in the atrias_gui namespace.
    nh.setParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
    nh.setParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
    nh.setParam("/atrias_gui/leg_for_kp", controllerDataOut.leg_for_kp);
    nh.setParam("/atrias_gui/leg_for_kd", controllerDataOut.leg_for_kd);
    nh.setParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
    nh.setParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
    nh.setParam("/atrias_gui/robot_ks", controllerDataOut.robot_ks);
    nh.setParam("/atrias_gui/robot_kt", controllerDataOut.robot_kt);
    nh.setParam("/atrias_gui/robot_kg", controllerDataOut.robot_kg);
    nh.setParam("/atrias_gui/left_hip_ang", controllerDataOut.left_hip_ang);
    nh.setParam("/atrias_gui/right_hip_ang", controllerDataOut.right_hip_ang);
    nh.setParam("/atrias_gui/left_toe_pos", controllerDataOut.left_toe_pos);
    nh.setParam("/atrias_gui/right_toe_pos", controllerDataOut.right_toe_pos);
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

} // setParameters

// guiUpdate ===================================================================
void guiUpdate() {
    controllerDataOut.leg_pos_kp = leg_pos_kp_spinbutton->get_value();
    controllerDataOut.leg_pos_kd = leg_pos_kd_spinbutton->get_value();
    controllerDataOut.leg_for_kp = leg_for_kp_spinbutton->get_value();
    controllerDataOut.leg_for_kd = leg_for_kd_spinbutton->get_value();
    controllerDataOut.hip_pos_kp = hip_pos_kp_spinbutton->get_value();
    controllerDataOut.hip_pos_kd = hip_pos_kd_spinbutton->get_value();
    controllerDataOut.robot_ks = robot_ks_spinbutton->get_value();
    controllerDataOut.robot_kt = robot_kt_spinbutton->get_value();
    controllerDataOut.robot_kg = robot_kg_spinbutton->get_value();
    controllerDataOut.left_hip_ang = left_hip_ang_spinbutton->get_value();
    controllerDataOut.right_hip_ang = right_hip_ang_spinbutton->get_value();
    controllerDataOut.left_toe_pos = left_toe_pos_spinbutton->get_value();
    controllerDataOut.right_toe_pos = right_toe_pos_spinbutton->get_value();
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
    controllerDataOut.constant_hip = constant_hip_radiobutton->get_active();
    controllerDataOut.constant_toe = constant_toe_radiobutton->get_active();
    controllerDataOut.left_leg_pos = left_leg_pos_radiobutton->get_active();
    controllerDataOut.right_leg_pos = right_leg_pos_radiobutton->get_active();
    controllerDataOut.left_leg_for = left_leg_for_radiobutton->get_active();
    controllerDataOut.right_leg_for = right_leg_for_radiobutton->get_active();
    controllerDataOut.left_leg_wave_for = left_leg_wave_for_radiobutton->get_active();
    controllerDataOut.right_leg_wave_for = right_leg_wave_for_radiobutton->get_active();

    pub.publish(controllerDataOut);

} // guiUpdate

// guiTakedown =================================================================
void guiTakedown() {
	// Stuff
} // guiTakedown

