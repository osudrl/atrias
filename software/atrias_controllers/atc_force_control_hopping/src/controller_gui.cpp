/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 */

#include <atc_force_control_hopping/controller_gui.h>

// guiInit =====================================================================
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {

	// Get widgets -------------------------------------------------------------
	gui->get_widget("appex_radiobutton", appex_radiobutton);
	gui->get_widget("terrain_radiobutton", terrain_radiobutton);
	gui->get_widget("two_leg_radiobutton", two_leg_radiobutton);
	gui->get_widget("alt_leg_radiobutton", alt_leg_radiobutton);
	gui->get_widget("left_leg_radiobutton", left_leg_radiobutton);
	gui->get_widget("right_leg_radiobutton", right_leg_radiobutton);
	gui->get_widget("hip_checkbutton", hip_checkbutton);
	gui->get_widget("left_toe_spinbutton", left_toe_spinbutton);
	gui->get_widget("right_toe_spinbutton", right_toe_spinbutton);
	gui->get_widget("stand_radiobutton", stand_radiobutton);
	gui->get_widget("hop_radiobutton", hop_radiobutton);
	gui->get_widget("slip_h_spinbutton", slip_h_spinbutton);
	gui->get_widget("stand_r0_spinbutton", stand_r0_spinbutton);
	gui->get_widget("slip_r0_spinbutton", slip_r0_spinbutton);
	gui->get_widget("slip_m_spinbutton", slip_m_spinbutton);
	gui->get_widget("leg_pos_kp_spinbutton", leg_pos_kp_spinbutton);
	gui->get_widget("leg_for_kp_spinbutton", leg_for_kp_spinbutton);
	gui->get_widget("leg_pos_kd_spinbutton", leg_pos_kd_spinbutton);
	gui->get_widget("leg_for_kd_spinbutton", leg_for_kd_spinbutton);
	gui->get_widget("hip_kp_spinbutton", hip_kp_spinbutton);
	gui->get_widget("hip_kd_spinbutton", hip_kd_spinbutton);
	gui->get_widget("robot_ks_spinbutton", robot_ks_spinbutton);
	gui->get_widget("robot_kt_spinbutton", robot_kt_spinbutton);
	gui->get_widget("robot_kg_spinbutton", robot_kg_spinbutton);
	gui->get_widget("deinit_checkbutton", deinit_checkbutton);
    
	// Set ranges --------------------------------------------------------------
	left_toe_spinbutton->set_range(2.0, 2.3);
	right_toe_spinbutton->set_range(2.3, 2.6);
	slip_h_spinbutton->set_range(0.0, 0.2);
	stand_r0_spinbutton->set_range(0.5, 0.95);
	slip_r0_spinbutton->set_range(0.5, 0.95);
	slip_m_spinbutton->set_range(50.0, 70.0);
	leg_pos_kp_spinbutton->set_range(0.0, 1000.0);
	leg_for_kp_spinbutton->set_range(0.0, 1500.0);
	leg_pos_kd_spinbutton->set_range(0.0, 100.0);
	leg_for_kd_spinbutton->set_range(0.0, 50.0);
	hip_kp_spinbutton->set_range(0.0, 300.0);
	hip_kd_spinbutton->set_range(0.0, 30.0);
	robot_ks_spinbutton->set_range(1000.0, 6000.0);
	robot_kt_spinbutton->set_range(0.0, 0.5);
	robot_kg_spinbutton->set_range(50.0, 50.0);
    
    // Set values --------------------------------------------------------------
	left_toe_spinbutton->set_value(2.15);
	right_toe_spinbutton->set_value(2.45);
	slip_h_spinbutton->set_value(0.05);
	stand_r0_spinbutton->set_value(0.9);
	slip_r0_spinbutton->set_value(0.85);
	slip_m_spinbutton->set_value(61.93);
	leg_pos_kp_spinbutton->set_value(500.0);
	leg_for_kp_spinbutton->set_value(1000.0);
	leg_pos_kd_spinbutton->set_value(25.0);
	leg_for_kd_spinbutton->set_value(8.0);
	hip_kp_spinbutton->set_value(150.0);
	hip_kd_spinbutton->set_value(8.0);
	robot_ks_spinbutton->set_value(4118.0);
	robot_kt_spinbutton->set_value(0.0987);
	robot_kg_spinbutton->set_value(50.0);

	// Set up subscriber and publisher.
    sub = nh.subscribe("atc_force_control_hopping_status", 0, controllerCallback);
    pub = nh.advertise<atc_force_control_hopping::controller_input>("atc_force_control_hopping_input", 0);
    return true;

} // guiInit

// controllerCallback ==========================================================
void controllerCallback(const atc_force_control_hopping::controller_status &status) {
    controllerDataIn = status;
    
} // controllerCallback

// getParameters ===============================================================
void getParameters() {
    // Get parameters in the atrias_gui namespace ------------------------------
	nh.getParam("/atrias_gui/left_toe", controllerDataOut.left_toe);
	nh.getParam("/atrias_gui/right_toe", controllerDataOut.right_toe);
	nh.getParam("/atrias_gui/slip_h", controllerDataOut.slip_h);
	nh.getParam("/atrias_gui/stand_r0", controllerDataOut.stand_r0);
	nh.getParam("/atrias_gui/slip_r0", controllerDataOut.slip_r0);
	nh.getParam("/atrias_gui/slip_m", controllerDataOut.slip_m);
	nh.getParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
	nh.getParam("/atrias_gui/leg_for_kp", controllerDataOut.leg_for_kp);
	nh.getParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
	nh.getParam("/atrias_gui/leg_for_kd", controllerDataOut.leg_for_kd);
	nh.getParam("/atrias_gui/hip_kp", controllerDataOut.hip_kp);
	nh.getParam("/atrias_gui/hip_kd", controllerDataOut.hip_kd);
	nh.getParam("/atrias_gui/robot_ks", controllerDataOut.robot_ks);
	nh.getParam("/atrias_gui/robot_kt", controllerDataOut.robot_kt);
	nh.getParam("/atrias_gui/robot_kg", controllerDataOut.robot_kg);

    // Configure the GUI -------------------------------------------------------
	left_toe_spinbutton->set_value(controllerDataOut.left_toe);
	right_toe_spinbutton->set_value(controllerDataOut.right_toe);
	slip_h_spinbutton->set_value(controllerDataOut.slip_h);
	stand_r0_spinbutton->set_value(controllerDataOut.stand_r0);
	slip_r0_spinbutton->set_value(controllerDataOut.slip_r0);
	slip_m_spinbutton->set_value(controllerDataOut.slip_m);
	leg_pos_kp_spinbutton->set_value(controllerDataOut.leg_pos_kp);
	leg_for_kp_spinbutton->set_value(controllerDataOut.leg_for_kp);
	leg_pos_kd_spinbutton->set_value(controllerDataOut.leg_pos_kd);
	leg_for_kd_spinbutton->set_value(controllerDataOut.leg_for_kd);
	hip_kp_spinbutton->set_value(controllerDataOut.hip_kp);
	hip_kd_spinbutton->set_value(controllerDataOut.hip_kd);
	robot_ks_spinbutton->set_value(controllerDataOut.robot_ks);
	robot_kt_spinbutton->set_value(controllerDataOut.robot_kt);
	robot_kg_spinbutton->set_value(controllerDataOut.robot_kg);

} // getParameters

// setParameters ===============================================================
void setParameters() {
	// Set parameters in the atrias_gui namespace ------------------------------
	nh.setParam("/atrias_gui/left_toe", controllerDataOut.left_toe);
	nh.setParam("/atrias_gui/right_toe", controllerDataOut.right_toe);
	nh.setParam("/atrias_gui/slip_h", controllerDataOut.slip_h);
	nh.setParam("/atrias_gui/stand_r0", controllerDataOut.stand_r0);
	nh.setParam("/atrias_gui/slip_r0", controllerDataOut.slip_r0);
	nh.setParam("/atrias_gui/slip_m", controllerDataOut.slip_m);
	nh.setParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
	nh.setParam("/atrias_gui/leg_for_kp", controllerDataOut.leg_for_kp);
	nh.setParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
	nh.setParam("/atrias_gui/leg_for_kd", controllerDataOut.leg_for_kd);
	nh.setParam("/atrias_gui/hip_kp", controllerDataOut.hip_kp);
	nh.setParam("/atrias_gui/hip_kd", controllerDataOut.hip_kd);
	nh.setParam("/atrias_gui/robot_ks", controllerDataOut.robot_ks);
	nh.setParam("/atrias_gui/robot_kt", controllerDataOut.robot_kt);
	nh.setParam("/atrias_gui/robot_kg", controllerDataOut.robot_kg);
	
} // setParameters

// guiUpdate ===================================================================
void guiUpdate() {
	// Update GUI --------------------------------------------------------------
	controllerDataOut.appex = appex_radiobutton->get_active();
	controllerDataOut.terrain = terrain_radiobutton->get_active();
	controllerDataOut.two_leg = two_leg_radiobutton->get_active();
	controllerDataOut.alt_leg = alt_leg_radiobutton->get_active();
	controllerDataOut.left_leg = left_leg_radiobutton->get_active();
	controllerDataOut.right_leg = right_leg_radiobutton->get_active();
	controllerDataOut.hip = hip_checkbutton->get_active();
	controllerDataOut.left_toe = left_toe_spinbutton->get_value();
	controllerDataOut.right_toe = right_toe_spinbutton->get_value();
	controllerDataOut.stand = stand_radiobutton->get_active();
	controllerDataOut.hop = hop_radiobutton->get_active();
	controllerDataOut.slip_h = slip_h_spinbutton->get_value();
	controllerDataOut.stand_r0 = stand_r0_spinbutton->get_value();
	controllerDataOut.slip_r0 = slip_r0_spinbutton->get_value();
	controllerDataOut.slip_m = slip_m_spinbutton->get_value();
	controllerDataOut.leg_pos_kp = leg_pos_kp_spinbutton->get_value();
	controllerDataOut.leg_for_kp = leg_for_kp_spinbutton->get_value();
	controllerDataOut.leg_pos_kd = leg_pos_kd_spinbutton->get_value();
	controllerDataOut.leg_for_kd = leg_for_kd_spinbutton->get_value();
	controllerDataOut.hip_kp = hip_kp_spinbutton->get_value();
	controllerDataOut.hip_kd = hip_kd_spinbutton->get_value();
	controllerDataOut.robot_ks = robot_ks_spinbutton->get_value();
	controllerDataOut.robot_kt = robot_kt_spinbutton->get_value();
	controllerDataOut.robot_kg = robot_kg_spinbutton->get_value();
	controllerDataOut.deinit = deinit_checkbutton->get_active();
    
    pub.publish(controllerDataOut);
    
} // guiUpdate

// guiTakedown =================================================================
void guiTakedown() {

} // guiTakedown

