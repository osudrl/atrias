/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 *  \brief Orocos Component code for the atc_force_control_demo controller.
 */

#include <atc_force_control_demo/controller_gui.h>

// guiInit =====================================================================
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {

	// Get widgets
    gui->get_widget("leg_pos_p_gain_spinbutton", leg_pos_p_gain_spinbutton);
    gui->get_widget("leg_pos_d_gain_spinbutton", leg_pos_d_gain_spinbutton);
    gui->get_widget("leg_force_p_gain_spinbutton", leg_force_p_gain_spinbutton);
    gui->get_widget("leg_force_d_gain_spinbutton", leg_force_d_gain_spinbutton);
    gui->get_widget("fx_spinbutton", fx_spinbutton);
    gui->get_widget("fz_spinbutton", fz_spinbutton);
    gui->get_widget("ampx_spinbutton", ampx_spinbutton);
    gui->get_widget("ampz_spinbutton", ampz_spinbutton);
    gui->get_widget("freqx_spinbutton", freqx_spinbutton);
    gui->get_widget("freqz_spinbutton", freqz_spinbutton);
    gui->get_widget("offsetx_spinbutton", offsetx_spinbutton);
    gui->get_widget("offsetz_spinbutton", offsetz_spinbutton);
    gui->get_widget("hip_angle_spinbutton", hip_angle_spinbutton);
    gui->get_widget("right_toe_spinbutton", left_toe_spinbutton);
    gui->get_widget("left_toe_spinbutton", right_toe_spinbutton);
    gui->get_widget("hip_p_gain_spinbutton", hip_p_gain_spinbutton);
    gui->get_widget("hip_d_gain_spinbutton", hip_d_gain_spinbutton);
    gui->get_widget("robot_spring_spinbutton", robot_spring_spinbutton);
    gui->get_widget("robot_motor_spinbutton", robot_motor_spinbutton);
    gui->get_widget("robot_gear_spinbutton", robot_gear_spinbutton);
    
    gui->get_widget("constant_force_checkbutton", constant_force_checkbutton);
    gui->get_widget("sinewave_force_checkbutton", sinewave_force_checkbutton);
    gui->get_widget("constant_hip_checkbutton", constant_hip_checkbutton);
    gui->get_widget("advanced_hip_checkbutton", advanced_hip_checkbutton);
    
	// Set ranges.
	leg_pos_p_gain_spinbutton->set_range(0.0, 5000.0);
	leg_pos_d_gain_spinbutton->set_range(0.0, 100.0);
	leg_force_p_gain_spinbutton->set_range(0.0, 5000.0);
	leg_force_d_gain_spinbutton->set_range(0.0, 100.0);
	fx_spinbutton->set_range(-1000.0, 1000.0);
	fz_spinbutton->set_range(-1000.0, 1000.0);
	ampx_spinbutton->set_range(-1000.0, 1000.0);
	ampz_spinbutton->set_range(-1000.0, 1000.0);
	freqx_spinbutton->set_range(0.0, 10.0);
	freqz_spinbutton->set_range(0.0, 10.0);
	offsetx_spinbutton->set_range(-1000.0, 1000.0);
	offsetz_spinbutton->set_range(-1000.0, 1000.0);
	hip_angle_spinbutton->set_range(M_PI, 2.0*M_PI);
	left_toe_spinbutton->set_range(1.0, 4.0);
	right_toe_spinbutton->set_range(1.0, 4.0);
	hip_p_gain_spinbutton->set_range(0.0, 500.0);
	hip_d_gain_spinbutton->set_range(0.0, 50.0);
	robot_spring_spinbutton->set_range(0.0, 5000.0);
	robot_motor_spinbutton->set_range(0.11, 0.11);
	robot_gear_spinbutton->set_range(50.0, 50.0);
	
	// Set default values
    leg_pos_p_gain_spinbutton->set_value(500.0);
    leg_pos_d_gain_spinbutton->set_value(50.0);
    leg_force_p_gain_spinbutton->set_value(4000.0);
    leg_force_d_gain_spinbutton->set_value(10.0);
    fx_spinbutton->set_value(0.0);
    fz_spinbutton->set_value(0.0);
    ampx_spinbutton->set_value(0.0);
    ampz_spinbutton->set_value(0.0);
    freqx_spinbutton->set_value(0.0);
    freqz_spinbutton->set_value(0.0);
    offsetx_spinbutton->set_value(0.0);
    offsetz_spinbutton->set_value(0.0);
    hip_angle_spinbutton->set_value(3.0*M_PI/2.0);
    left_toe_spinbutton->set_value(2.00);
    right_toe_spinbutton->set_value(2.40);
    hip_p_gain_spinbutton->set_value(100.0);
    hip_d_gain_spinbutton->set_value(5.0);
    robot_spring_spinbutton->set_value(4118.0);
    robot_motor_spinbutton->set_value(0.11);
    robot_gear_spinbutton->set_value(50.0);

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
    nh.getParam("/atrias_gui/leg_pos_p_gain", controllerDataOut.leg_pos_p_gain);
    nh.getParam("/atrias_gui/leg_pos_d_gain", controllerDataOut.leg_pos_d_gain);
    nh.getParam("/atrias_gui/leg_force_p_gain", controllerDataOut.leg_force_p_gain);
    nh.getParam("/atrias_gui/leg_force_d_gain", controllerDataOut.leg_force_d_gain);
    nh.getParam("/atrias_gui/fx", controllerDataOut.fx);
    nh.getParam("/atrias_gui/fz", controllerDataOut.fz);
    nh.getParam("/atrias_gui/ampx", controllerDataOut.ampx);
    nh.getParam("/atrias_gui/ampz", controllerDataOut.ampz);
    nh.getParam("/atrias_gui/freqx", controllerDataOut.freqx);
    nh.getParam("/atrias_gui/freqz", controllerDataOut.freqz);
    nh.getParam("/atrias_gui/offsetx", controllerDataOut.offsetx);
    nh.getParam("/atrias_gui/offsetz", controllerDataOut.offsetz);
    nh.getParam("/atrias_gui/hip_angle", controllerDataOut.hip_angle);
    nh.getParam("/atrias_gui/left_toe", controllerDataOut.left_toe);
    nh.getParam("/atrias_gui/right_toe", controllerDataOut.right_toe);
    nh.getParam("/atrias_gui/hip_p_gain", controllerDataOut.hip_p_gain);
    nh.getParam("/atrias_gui/hip_d_gain", controllerDataOut.hip_d_gain);
    nh.getParam("/atrias_gui/robot_spring", controllerDataOut.robot_spring);
    nh.getParam("/atrias_gui/robot_motor", controllerDataOut.robot_motor);
    nh.getParam("/atrias_gui/robot_gear", controllerDataOut.robot_gear);

    // Configure the GUI.
    leg_pos_p_gain_spinbutton->set_value(controllerDataOut.leg_pos_p_gain);
    leg_pos_d_gain_spinbutton->set_value(controllerDataOut.leg_pos_d_gain);
    leg_force_p_gain_spinbutton->set_value(controllerDataOut.leg_force_p_gain);
    leg_force_d_gain_spinbutton->set_value(controllerDataOut.leg_force_d_gain);
    fx_spinbutton->set_value(controllerDataOut.fx);
    fz_spinbutton->set_value(controllerDataOut.fz);
    ampx_spinbutton->set_value(controllerDataOut.ampx);
    ampz_spinbutton->set_value(controllerDataOut.ampz);
    freqx_spinbutton->set_value(controllerDataOut.freqx);
    freqz_spinbutton->set_value(controllerDataOut.freqz);
    offsetx_spinbutton->set_value(controllerDataOut.offsetx);
    offsetz_spinbutton->set_value(controllerDataOut.offsetz);
    hip_angle_spinbutton->set_value(controllerDataOut.hip_angle);
    left_toe_spinbutton->set_value(controllerDataOut.left_toe);
    right_toe_spinbutton->set_value(controllerDataOut.right_toe);
    hip_p_gain_spinbutton->set_value(controllerDataOut.hip_p_gain);
    hip_d_gain_spinbutton->set_value(controllerDataOut.hip_d_gain);
    robot_spring_spinbutton->set_value(controllerDataOut.robot_spring);
    robot_motor_spinbutton->set_value(controllerDataOut.robot_motor);
    robot_gear_spinbutton->set_value(controllerDataOut.robot_gear);
    
    

} // getParameters

// setParameters ===============================================================
void setParameters() {
	// Set parameters in the atrias_gui namespace.
    nh.setParam("/atrias_gui/leg_pos_p_gain", controllerDataOut.leg_pos_p_gain);
    nh.setParam("/atrias_gui/leg_pos_d_gain", controllerDataOut.leg_pos_d_gain);
    nh.setParam("/atrias_gui/leg_force_p_gain", controllerDataOut.leg_force_p_gain);
    nh.setParam("/atrias_gui/leg_force_d_gain", controllerDataOut.leg_force_d_gain);
    nh.setParam("/atrias_gui/fx", controllerDataOut.fx);
    nh.setParam("/atrias_gui/fz", controllerDataOut.fz);
    nh.setParam("/atrias_gui/ampx", controllerDataOut.ampx);
    nh.setParam("/atrias_gui/ampz", controllerDataOut.ampz);
    nh.setParam("/atrias_gui/freqx", controllerDataOut.freqx);
    nh.setParam("/atrias_gui/freqz", controllerDataOut.freqz);
    nh.setParam("/atrias_gui/offsetx", controllerDataOut.offsetx);
    nh.setParam("/atrias_gui/offsetz", controllerDataOut.offsetz);
    nh.setParam("/atrias_gui/hip_angle", controllerDataOut.hip_angle);
	nh.setParam("/atrias_gui/left_toe", controllerDataOut.left_toe);
    nh.setParam("/atrias_gui/right_toe", controllerDataOut.right_toe);
	nh.setParam("/atrias_gui/hip_p_gain", controllerDataOut.hip_p_gain);
    nh.setParam("/atrias_gui/hip_d_gain", controllerDataOut.hip_d_gain);
    nh.setParam("/atrias_gui/robot_spring", controllerDataOut.robot_spring);
    nh.setParam("/atrias_gui/robot_motor", controllerDataOut.robot_motor);
    nh.setParam("/atrias_gui/robot_gear", controllerDataOut.robot_gear);

} // setParameters

// guiUpdate ===================================================================
void guiUpdate() {
    controllerDataOut.leg_pos_p_gain = leg_pos_p_gain_spinbutton->get_value();
    controllerDataOut.leg_pos_d_gain = leg_pos_d_gain_spinbutton->get_value();
    controllerDataOut.leg_force_p_gain = leg_force_p_gain_spinbutton->get_value();
    controllerDataOut.leg_force_d_gain = leg_force_d_gain_spinbutton->get_value();
    controllerDataOut.fx = fx_spinbutton->get_value();
    controllerDataOut.fz = fz_spinbutton->get_value();
    controllerDataOut.ampx = ampx_spinbutton->get_value();
    controllerDataOut.ampz = ampz_spinbutton->get_value();
    controllerDataOut.freqx = freqx_spinbutton->get_value();
    controllerDataOut.freqz = freqz_spinbutton->get_value();
    controllerDataOut.offsetx = offsetx_spinbutton->get_value();
    controllerDataOut.offsetz = offsetz_spinbutton->get_value();
    controllerDataOut.hip_angle = hip_angle_spinbutton->get_value();
    controllerDataOut.left_toe = left_toe_spinbutton->get_value();
    controllerDataOut.right_toe = right_toe_spinbutton->get_value();
    controllerDataOut.hip_p_gain = hip_p_gain_spinbutton->get_value();
    controllerDataOut.hip_d_gain = hip_d_gain_spinbutton->get_value();
    controllerDataOut.robot_spring = robot_spring_spinbutton->get_value();
    controllerDataOut.robot_motor = robot_motor_spinbutton->get_value();
    controllerDataOut.robot_gear = robot_gear_spinbutton->get_value();
    
    controllerDataOut.constant_force = constant_force_checkbutton->get_active();
    controllerDataOut.sinewave_force = sinewave_force_checkbutton->get_active();
    controllerDataOut.constant_hip = constant_hip_checkbutton->get_active();
    controllerDataOut.advanced_hip = advanced_hip_checkbutton->get_active();

    pub.publish(controllerDataOut);

} // guiUpdate

// guiTakedown =================================================================
void guiTakedown() {
	// Stuff
} // guiTakedown

