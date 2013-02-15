/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 */

#include <atc_vertical_force_control_hopping/controller_gui.h>

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
    gui->get_widget("left_toe_pos_spinbutton", left_toe_pos_spinbutton);
    gui->get_widget("right_toe_pos_spinbutton", right_toe_pos_spinbutton);
    gui->get_widget("left_leg_len_spinbutton", left_leg_len_spinbutton);
    gui->get_widget("right_leg_len_spinbutton", right_leg_len_spinbutton);
    gui->get_widget("left_leg_ang_spinbutton", left_leg_ang_spinbutton);
    gui->get_widget("right_leg_ang_spinbutton", right_leg_ang_spinbutton);
    gui->get_widget("slip_h_spinbutton", slip_h_spinbutton);
    gui->get_widget("slip_l0_spinbutton", slip_l0_spinbutton);
    gui->get_widget("slip_m_spinbutton", slip_m_spinbutton);
    gui->get_widget("slip_k_spinbutton", slip_k_spinbutton);
    gui->get_widget("hip_contr_checkbutton", hip_contr_checkbutton);
    gui->get_widget("leg_pos_contr_radiobutton", leg_pos_contr_radiobutton);
    gui->get_widget("leg_slip_contr_radiobutton", leg_slip_contr_radiobutton);
    gui->get_widget("left_leg_hop_radiobutton", left_leg_hop_radiobutton);
    gui->get_widget("right_leg_hop_radiobutton", right_leg_hop_radiobutton);
    gui->get_widget("two_leg_hop_radiobutton", two_leg_hop_radiobutton);
    gui->get_widget("alt_leg_hop_radiobutton", alt_leg_hop_radiobutton);
    gui->get_widget("debug1_checkbutton", debug1_checkbutton);
    
	// Set ranges.
    leg_pos_kp_spinbutton->set_range(0.0, 1000.0);
    leg_pos_kd_spinbutton->set_range(0.0, 100.0);
    leg_for_kp_spinbutton->set_range(0.0, 5000.0);
    leg_for_kd_spinbutton->set_range(0.0, 100.0);
    hip_pos_kp_spinbutton->set_range(0.0, 250.0);
    hip_pos_kd_spinbutton->set_range(0.0, 25.0);
    robot_ks_spinbutton->set_range(0.0, 10000.0);
    robot_kt_spinbutton->set_range(0.0, 1.0);
    robot_kg_spinbutton->set_range(50.0, 50.0);
    left_toe_pos_spinbutton->set_range(2.0, 2.3);
    right_toe_pos_spinbutton->set_range(2.3, 2.6);
    left_leg_len_spinbutton->set_range(0.45, 0.925);
    right_leg_len_spinbutton->set_range(0.45, 0.925);
    left_leg_ang_spinbutton->set_range(M_PI/4.0, 3.0*M_PI/4.0);
    right_leg_ang_spinbutton->set_range(M_PI/4.0, 3.0*M_PI/4.0);
    slip_h_spinbutton->set_range(0.0, 0.2);
    slip_l0_spinbutton->set_range(0.5, 0.9);
    slip_m_spinbutton->set_range(0.0, 100.0);
    slip_k_spinbutton->set_range(0.0, 25000.0);
    
    // Set values
    leg_pos_kp_spinbutton->set_value(500.0);
    leg_pos_kd_spinbutton->set_value(25.0);
    leg_for_kp_spinbutton->set_value(5000.0);
    leg_for_kd_spinbutton->set_value(25.0);
    hip_pos_kp_spinbutton->set_value(100.0);
    hip_pos_kd_spinbutton->set_value(7.5);
    robot_ks_spinbutton->set_value(4118.0);//4118.0
    robot_kt_spinbutton->set_value(0.0987);
    robot_kg_spinbutton->set_value(50.0);
    left_toe_pos_spinbutton->set_value(2.15);
    right_toe_pos_spinbutton->set_value(2.45);
    left_leg_len_spinbutton->set_value(0.85);
    right_leg_len_spinbutton->set_value(0.85);
    left_leg_ang_spinbutton->set_value(M_PI/2.0);
    right_leg_ang_spinbutton->set_value(M_PI/2.0);
    slip_h_spinbutton->set_value(0.05);
    slip_l0_spinbutton->set_value(0.85);
    slip_m_spinbutton->set_value(60.0);
    slip_k_spinbutton->set_value(16000.0);

	// Set up subscriber and publisher.
	sub = nh.subscribe("atc_vertical_force_control_hopping_status", 0, controllerCallback);
	pub = nh.advertise<atc_vertical_force_control_hopping::controller_input>("atc_vertical_force_control_hopping_input", 0);

	return true;

} // guiInit

// controllerCallback ==========================================================
void controllerCallback(const atc_vertical_force_control_hopping::controller_status &status) {
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
    nh.getParam("/atrias_gui/left_toe_pos", controllerDataOut.left_toe_pos);
    nh.getParam("/atrias_gui/right_toe_pos", controllerDataOut.right_toe_pos);
    nh.getParam("/atrias_gui/left_leg_len", controllerDataOut.left_leg_len);
    nh.getParam("/atrias_gui/right_leg_len", controllerDataOut.right_leg_len);
    nh.getParam("/atrias_gui/left_leg_ang", controllerDataOut.left_leg_ang);
    nh.getParam("/atrias_gui/right_leg_ang", controllerDataOut.right_leg_ang);
    nh.getParam("/atrias_gui/slip_h", controllerDataOut.slip_h);
    nh.getParam("/atrias_gui/slip_l0", controllerDataOut.slip_l0);
    nh.getParam("/atrias_gui/slip_m", controllerDataOut.slip_m);
    nh.getParam("/atrias_gui/slip_k", controllerDataOut.slip_k);

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
    left_toe_pos_spinbutton->set_value(controllerDataOut.left_toe_pos);
    right_toe_pos_spinbutton->set_value(controllerDataOut.right_toe_pos);
    left_leg_len_spinbutton->set_value(controllerDataOut.left_leg_len);
    right_leg_len_spinbutton->set_value(controllerDataOut.right_leg_len);
    left_leg_ang_spinbutton->set_value(controllerDataOut.left_leg_ang);
    right_leg_ang_spinbutton->set_value(controllerDataOut.right_leg_ang);
    slip_h_spinbutton->set_value(controllerDataOut.slip_h);
    slip_l0_spinbutton->set_value(controllerDataOut.slip_l0);
    slip_m_spinbutton->set_value(controllerDataOut.slip_m);
    slip_k_spinbutton->set_value(controllerDataOut.slip_k);

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
    nh.setParam("/atrias_gui/left_toe_pos", controllerDataOut.left_toe_pos);
    nh.setParam("/atrias_gui/right_toe_pos", controllerDataOut.right_toe_pos);
    nh.setParam("/atrias_gui/left_leg_len", controllerDataOut.left_leg_len);
    nh.setParam("/atrias_gui/right_leg_len", controllerDataOut.right_leg_len);
    nh.setParam("/atrias_gui/left_leg_ang", controllerDataOut.left_leg_ang);
    nh.setParam("/atrias_gui/right_leg_ang", controllerDataOut.right_leg_ang);
    nh.setParam("/atrias_gui/slip_h", controllerDataOut.slip_h);
    nh.setParam("/atrias_gui/slip_l0", controllerDataOut.slip_l0);
    nh.setParam("/atrias_gui/slip_m", controllerDataOut.slip_m);
    nh.setParam("/atrias_gui/slip_k", controllerDataOut.slip_k);

} // setParameters

// guiUpdate ===================================================================
void guiUpdate() {
	// Update GUI
    controllerDataOut.leg_pos_kp = leg_pos_kp_spinbutton->get_value();
    controllerDataOut.leg_pos_kd = leg_pos_kd_spinbutton->get_value();
    controllerDataOut.leg_for_kp = leg_for_kp_spinbutton->get_value();
    controllerDataOut.leg_for_kd = leg_for_kd_spinbutton->get_value();
    controllerDataOut.hip_pos_kp = hip_pos_kp_spinbutton->get_value();
    controllerDataOut.hip_pos_kd = hip_pos_kd_spinbutton->get_value();
    controllerDataOut.robot_ks = robot_ks_spinbutton->get_value();
    controllerDataOut.robot_kt = robot_kt_spinbutton->get_value();
    controllerDataOut.robot_kg = robot_kg_spinbutton->get_value();
    controllerDataOut.left_toe_pos = left_toe_pos_spinbutton->get_value();
    controllerDataOut.right_toe_pos = right_toe_pos_spinbutton->get_value();
    controllerDataOut.left_leg_len = left_leg_len_spinbutton->get_value();
    controllerDataOut.right_leg_len = right_leg_len_spinbutton->get_value();
    controllerDataOut.left_leg_ang = left_leg_ang_spinbutton->get_value();
    controllerDataOut.right_leg_ang = right_leg_ang_spinbutton->get_value();
    controllerDataOut.slip_h = slip_h_spinbutton->get_value();
    controllerDataOut.slip_l0 = slip_l0_spinbutton->get_value();
    controllerDataOut.slip_m = slip_m_spinbutton->get_value();
    controllerDataOut.slip_k = slip_k_spinbutton->get_value();
    controllerDataOut.hip_contr = hip_contr_checkbutton->get_active();
    controllerDataOut.leg_pos_contr = leg_pos_contr_radiobutton->get_active();
    controllerDataOut.leg_slip_contr = leg_slip_contr_radiobutton->get_active();
    controllerDataOut.left_leg_hop = left_leg_hop_radiobutton->get_active();
    controllerDataOut.right_leg_hop = right_leg_hop_radiobutton->get_active();
    controllerDataOut.two_leg_hop = two_leg_hop_radiobutton->get_active();
    controllerDataOut.alt_leg_hop = alt_leg_hop_radiobutton->get_active();
    controllerDataOut.debug1 = debug1_checkbutton->get_active();
    
    pub.publish(controllerDataOut);

} // guiUpdate

// guiTakedown =================================================================
void guiTakedown() {
	// Stuff
} // guiTakedown

