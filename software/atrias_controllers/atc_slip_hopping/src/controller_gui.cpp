/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 */

#include <atc_slip_hopping/controller_gui.h>

// guiInit
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {

	// Get widgets
    gui->get_widget("slip_spring_spinbutton", slip_spring_spinbutton);
    gui->get_widget("standing_leg_spinbutton", standing_leg_spinbutton);
    gui->get_widget("hop_height_spinbutton", hop_height_spinbutton);
    gui->get_widget("slip_leg_spinbutton", slip_leg_spinbutton);
    gui->get_widget("leg_pos_kp_spinbutton", leg_pos_kp_spinbutton);
    gui->get_widget("leg_for_kp_spinbutton", leg_for_kp_spinbutton);
    gui->get_widget("leg_for_kd_spinbutton", leg_for_kd_spinbutton);
    gui->get_widget("leg_pos_kd_spinbutton", leg_pos_kd_spinbutton);
    gui->get_widget("hip_pos_kp_spinbutton", hip_pos_kp_spinbutton);
    gui->get_widget("hip_pos_kd_spinbutton", hip_pos_kd_spinbutton);
    gui->get_widget("main_controller_combobox", main_controller_combobox);
    gui->get_widget("spring_type_combobox", spring_type_combobox);
    gui->get_widget("force_type_combobox", force_type_combobox);
    gui->get_widget("stance_controller_combobox", stance_controller_combobox);
    gui->get_widget("hop_type_combobox", hop_type_combobox);

	// Set ranges
    slip_spring_spinbutton->set_range(0.0, 50000.0);
    standing_leg_spinbutton->set_range(0.5, 0.95);
    hop_height_spinbutton->set_range(0.0, 0.25);
    slip_leg_spinbutton->set_range(0.5, 0.95);
    leg_pos_kp_spinbutton->set_range(0.0, 2000.0);
    leg_for_kp_spinbutton->set_range(0.0, 2000.0);
    leg_for_kd_spinbutton->set_range(0.0, 50.0);
    leg_pos_kd_spinbutton->set_range(0.0, 50.0);
    hip_pos_kp_spinbutton->set_range(0.0, 250.0);
    hip_pos_kd_spinbutton->set_range(0.0, 25.0);

	// Set increments
    slip_spring_spinbutton->set_increments(100.0, 0.0);
    standing_leg_spinbutton->set_increments(0.01, 0.0);
    hop_height_spinbutton->set_increments(0.01, 0.0);
    slip_leg_spinbutton->set_increments(0.01, 0.0);
    leg_pos_kp_spinbutton->set_increments(10.0, 0.0);
    leg_for_kp_spinbutton->set_increments(10.0, 0.0);
    leg_for_kd_spinbutton->set_increments(1.0, 0.0);
    leg_pos_kd_spinbutton->set_increments(1.0, 0.0);
    hip_pos_kp_spinbutton->set_increments(10.0, 0.0);
    hip_pos_kd_spinbutton->set_increments(1.0, 0.0);
    
    // Set values
    slip_spring_spinbutton->set_value(28000.0);
    standing_leg_spinbutton->set_value(0.90);
    hop_height_spinbutton->set_value(0.05);
    slip_leg_spinbutton->set_value(0.85);
    leg_pos_kp_spinbutton->set_value(500.0);
    leg_for_kp_spinbutton->set_value(1000.0);
    leg_for_kd_spinbutton->set_value(8.0);
    leg_pos_kd_spinbutton->set_value(25.0);
    hip_pos_kp_spinbutton->set_value(100.0);
    hip_pos_kd_spinbutton->set_value(8.0);
    
	// Set up subscriber and publisher.
    sub = nh.subscribe("controller_status", 0, controllerCallback);
    pub = nh.advertise<atc_slip_hopping::controller_input>("controller_input", 0);
    return true;

}


// controllerCallback
void controllerCallback(const atc_slip_hopping::controller_status &status) {

    controllerDataIn = status;
    
} // controllerCallback


// getParameters
void getParameters() {

    // Get parameters in the atrias_gui namespace
    nh.getParam("/atrias_gui/slip_spring", controllerDataOut.slip_spring);
    nh.getParam("/atrias_gui/standing_leg", controllerDataOut.standing_leg);
    nh.getParam("/atrias_gui/hop_height", controllerDataOut.hop_height);
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
    int spring_type;
    nh.getParam("/atrias_gui/spring_type", spring_type);
    controllerDataOut.spring_type = (uint8_t)spring_type;
    int force_type;
    nh.getParam("/atrias_gui/force_type", force_type);
    controllerDataOut.force_type = (uint8_t)force_type;
    int stance_controller;
    nh.getParam("/atrias_gui/stance_controller", stance_controller);
    controllerDataOut.stance_controller = (uint8_t)stance_controller;
    int hop_type;
    nh.getParam("/atrias_gui/hop_type", hop_type);
    controllerDataOut.hop_type = (uint8_t)hop_type;

    // Configure the GUI
    slip_spring_spinbutton->set_value(controllerDataOut.slip_spring);
    standing_leg_spinbutton->set_value(controllerDataOut.standing_leg);
    hop_height_spinbutton->set_value(controllerDataOut.hop_height);
    slip_leg_spinbutton->set_value(controllerDataOut.slip_leg);
    leg_pos_kp_spinbutton->set_value(controllerDataOut.leg_pos_kp);
    leg_for_kp_spinbutton->set_value(controllerDataOut.leg_for_kp);
    leg_for_kd_spinbutton->set_value(controllerDataOut.leg_for_kd);
    leg_pos_kd_spinbutton->set_value(controllerDataOut.leg_pos_kd);
    hip_pos_kp_spinbutton->set_value(controllerDataOut.hip_pos_kp);
    hip_pos_kd_spinbutton->set_value(controllerDataOut.hip_pos_kd);
    main_controller_combobox->set_active(controllerDataOut.main_controller);
    spring_type_combobox->set_active(controllerDataOut.spring_type);
    force_type_combobox->set_active(controllerDataOut.force_type);
    stance_controller_combobox->set_active(controllerDataOut.stance_controller);
    hop_type_combobox->set_active(controllerDataOut.hop_type);

} // getParameters


// setParameters
void setParameters() {
    nh.setParam("/atrias_gui/slip_spring", controllerDataOut.slip_spring);
    nh.setParam("/atrias_gui/standing_leg", controllerDataOut.standing_leg);
    nh.setParam("/atrias_gui/hop_height", controllerDataOut.hop_height);
    nh.setParam("/atrias_gui/slip_leg", controllerDataOut.slip_leg);
    nh.setParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
    nh.setParam("/atrias_gui/leg_for_kp", controllerDataOut.leg_for_kp);
    nh.setParam("/atrias_gui/leg_for_kd", controllerDataOut.leg_for_kd);
    nh.setParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
    nh.setParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
    nh.setParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
    nh.setParam("/atrias_gui/main_controller", controllerDataOut.main_controller);
    nh.setParam("/atrias_gui/spring_type", controllerDataOut.spring_type);
    nh.setParam("/atrias_gui/force_type", controllerDataOut.force_type);
    nh.setParam("/atrias_gui/stance_controller", controllerDataOut.stance_controller);
    nh.setParam("/atrias_gui/hop_type", controllerDataOut.hop_type);
    
} // setParameters


// guiUpdate
void guiUpdate() {

	// Update GUI
    controllerDataOut.slip_spring = slip_spring_spinbutton->get_value();
    controllerDataOut.standing_leg = standing_leg_spinbutton->get_value();
    controllerDataOut.hop_height = hop_height_spinbutton->get_value();
    controllerDataOut.slip_leg = slip_leg_spinbutton->get_value();
    controllerDataOut.leg_pos_kp = leg_pos_kp_spinbutton->get_value();
    controllerDataOut.leg_for_kp = leg_for_kp_spinbutton->get_value();
    controllerDataOut.leg_for_kd = leg_for_kd_spinbutton->get_value();
    controllerDataOut.leg_pos_kd = leg_pos_kd_spinbutton->get_value();
    controllerDataOut.hip_pos_kp = hip_pos_kp_spinbutton->get_value();
    controllerDataOut.hip_pos_kd = hip_pos_kd_spinbutton->get_value();
    controllerDataOut.main_controller = (uint8_t)main_controller_combobox->get_active_row_number();
    controllerDataOut.spring_type = (uint8_t)spring_type_combobox->get_active_row_number();
    controllerDataOut.force_type = (uint8_t)force_type_combobox->get_active_row_number();
    controllerDataOut.stance_controller = (uint8_t)stance_controller_combobox->get_active_row_number();
    controllerDataOut.hop_type = (uint8_t)hop_type_combobox->get_active_row_number();
    
    // Publish
    pub.publish(controllerDataOut);
    
} // guiUpdate

// guiTakedown
void guiTakedown() {
	// Stuff
} // guiTakedown

