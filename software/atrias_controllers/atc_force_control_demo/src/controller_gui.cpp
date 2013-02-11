/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 */

#include <atc_force_control_demo/controller_gui.h>

// guiInit =====================================================================
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {

	// Get widgets

	// Set ranges.


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
    nh.getParam("/atrias_gui/torque_A", torque_A_param);
    nh.getParam("/atrias_gui/torque_B", torque_B_param);
    nh.getParam("/atrias_gui/torque_hip", torque_hip_param);

    // Configure the GUI.
    torque_A_hscale->set_value(torque_A_param);
    torque_B_hscale->set_value(torque_B_param);
    torque_hip_hscale->set_value(torque_hip_param);

} // getParameters

// setParameters ===============================================================
void setParameters() {
    nh.setParam("/atrias_gui/torque_A", torque_A_param);
    nh.setParam("/atrias_gui/torque_B", torque_B_param);
    nh.setParam("/atrias_gui/torque_hip", torque_hip_param);

} // setParameters

// guiUpdate ===================================================================
void guiUpdate() {
    controllerDataOut.des_motor_torque_A   = torque_A_param   = torque_A_hscale->get_value();
    controllerDataOut.des_motor_torque_B   = torque_B_param   = torque_B_hscale->get_value();
    controllerDataOut.des_motor_torque_hip = torque_hip_param = torque_hip_hscale->get_value();
    pub.publish(controllerDataOut);

} // guiUpdate

// guiTakedown =================================================================
void guiTakedown() {
	// Stuff
} // guiTakedown

