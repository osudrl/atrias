/*
 * controller_gui.cpp
 *
 * Motor Torque Controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <atc_motor_position/controller_gui.h>

bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    gui->get_widget("position_left_A_hscale",    position_left_A_hscale);
    gui->get_widget("position_left_B_hscale",    position_left_B_hscale);
    gui->get_widget("position_left_hip_hscale",  position_left_hip_hscale);
    gui->get_widget("position_right_A_hscale",   position_right_A_hscale);
    gui->get_widget("position_right_B_hscale",   position_right_B_hscale);
    gui->get_widget("position_right_hip_hscale", position_right_hip_hscale);
    gui->get_widget("position_leg_motor_p_hscale", position_leg_motor_p_spinbutton);
    gui->get_widget("position_leg_motor_d_hscale", position_leg_motor_d_spinbutton);
    gui->get_widget("position_hip_motor_p_hscale", position_hip_motor_p_spinbutton);
    gui->get_widget("position_hip_motor_d_hscale", position_hip_motor_d_spinbutton);
    gui->get_widget("set_leg_motor_position_checkbutton", set_leg_motor_position_checkbutton);
    gui->get_widget("set_hip_motor_position_checkbutton", set_hip_motor_position_checkbutton);

    if (position_left_A_hscale && position_left_B_hscale && position_left_hip_hscale &&
        position_right_A_hscale && position_right_B_hscale && position_right_hip_hscale &&
        position_leg_motor_p_spinbutton && position_leg_motor_d_spinbutton &&
        position_hip_motor_p_spinbutton && position_hip_motor_d_spinbutton &&
        set_leg_motor_position_checkbutton && set_hip_motor_position_checkbutton) {
        // Set ranges.
        position_left_A_hscale->set_range(-1 * (M_PI / 4.), M_PI);
        position_left_B_hscale->set_range(0, (5. * M_PI) / 4.);
        position_left_hip_hscale->set_range(0, 0);   // TODO: Calculate range.
        position_right_A_hscale->set_range(-1 * (M_PI / 4.), M_PI);
        position_right_B_hscale->set_range(0, (5. * M_PI) / 4.);
        position_right_hip_hscale->set_range(0, 0);   // TODO: Calculate range.
	position_leg_motor_p_spinbutton->set_range(0., 10000.);
        position_leg_motor_d_spinbutton->set_range(0., 500.);
	position_hip_motor_p_spinbutton->set_range(0., 0.);   // TODO: Calculate range.
        position_hip_motor_d_spinbutton->set_range(0., 0.);   // TODO: Calculate range.

        // Set up subscriber and publisher.
        sub = nh.subscribe("atc_motor_position_status", 0, controllerCallback);
        pub = nh.advertise<atc_motor_position::controller_input>("atc_motor_position_input", 0);

        return true;
    }

    return false;
}

void controllerCallback(const atc_motor_position::controller_status &status) {
    controllerDataIn = status;
}

//void getParam(std::string name, double param, Gtk::Object* gtkObj) {
//    nh.getParam(name, param);
//    gtkObj->set_value(param);
//}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
    // Get parameters in the atrias_gui namespace.
    nh.getParam("/atrias_gui/des_motor_ang_left_A", controllerDataOut.des_motor_ang_left_A);
    nh.getParam("/atrias_gui/des_motor_ang_left_B", controllerDataOut.des_motor_ang_left_B);
    nh.getParam("/atrias_gui/des_motor_ang_left_hip", controllerDataOut.des_motor_ang_left_hip);
    nh.getParam("/atrias_gui/des_motor_ang_right_A", controllerDataOut.des_motor_ang_right_A);
    nh.getParam("/atrias_gui/des_motor_ang_right_B", controllerDataOut.des_motor_ang_right_B);
    nh.getParam("/atrias_gui/des_motor_ang_right_hip", controllerDataOut.des_motor_ang_right_hip);
    nh.getParam("/atrias_gui/leg_motor_p_gain", controllerDataOut.leg_motor_p_gain);
    nh.getParam("/atrias_gui/leg_motor_d_gain", controllerDataOut.leg_motor_d_gain);
    nh.getParam("/atrias_gui/hip_motor_p_gain", controllerDataOut.hip_motor_p_gain);
    nh.getParam("/atrias_gui/hip_motor_d_gain", controllerDataOut.hip_motor_d_gain);

    // Configure the GUI.
    position_left_A_hscale->set_value(controllerDataOut.des_motor_ang_left_A);
    position_left_B_hscale->set_value(controllerDataOut.des_motor_ang_left_B);
    position_left_hip_hscale->set_value(controllerDataOut.des_motor_ang_left_hip);
    position_right_A_hscale->set_value(controllerDataOut.des_motor_ang_right_A);
    position_right_B_hscale->set_value(controllerDataOut.des_motor_ang_right_B);
    position_right_hip_hscale->set_value(controllerDataOut.des_motor_ang_right_hip);
    position_leg_motor_p_spinbutton->set_value(controllerDataOut.leg_motor_p_gain);
    position_leg_motor_d_spinbutton->set_value(controllerDataOut.leg_motor_d_gain);
    position_hip_motor_p_spinbutton->set_value(controllerDataOut.hip_motor_p_gain);
    position_hip_motor_d_spinbutton->set_value(controllerDataOut.hip_motor_d_gain);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    nh.setParam("/atrias_gui/des_motor_ang_left_A", controllerDataOut.des_motor_ang_left_A);
    nh.setParam("/atrias_gui/des_motor_ang_left_B", controllerDataOut.des_motor_ang_left_B);
    nh.setParam("/atrias_gui/des_motor_ang_left_hip", controllerDataOut.des_motor_ang_left_hip);
    nh.setParam("/atrias_gui/des_motor_ang_right_A", controllerDataOut.des_motor_ang_right_A);
    nh.setParam("/atrias_gui/des_motor_ang_right_B", controllerDataOut.des_motor_ang_right_B);
    nh.setParam("/atrias_gui/des_motor_ang_right_hip", controllerDataOut.des_motor_ang_right_hip);
    nh.setParam("/atrias_gui/leg_motor_p_gain", controllerDataOut.leg_motor_p_gain);
    nh.setParam("/atrias_gui/leg_motor_d_gain", controllerDataOut.leg_motor_d_gain);
    nh.setParam("/atrias_gui/hip_motor_p_gain", controllerDataOut.hip_motor_p_gain);
    nh.setParam("/atrias_gui/hip_motor_d_gain", controllerDataOut.hip_motor_d_gain);
}

void guiUpdate() {
    if ((!controllerDataIn.isEnabled) && set_position_checkbutton->get_active()) {
    	//We want to update the sliders with the current position
    	position_left_A_hscale->set_value(controllerDataIn.motorPositionLeftA);
    	position_left_B_hscale->set_value(controllerDataIn.motorPositionLeftB);
    	position_left_hip_hscale->set_value(controllerDataIn.motorPositionLeftHip);
    	position_right_A_hscale->set_value(controllerDataIn.motorPositionRightA);
    	position_right_B_hscale->set_value(controllerDataIn.motorPositionRightB);
    	position_right_hip_hscale->set_value(controllerDataIn.motorPositionRightHip);
    }

    controllerDataOut.des_motor_ang_left_A    = position_left_A_hscale->get_value();
    controllerDataOut.des_motor_ang_left_B    = position_left_B_hscale->get_value();
    controllerDataOut.des_motor_ang_left_hip  = position_left_hip_hscale->get_value();
    controllerDataOut.des_motor_ang_right_A   = position_right_A_hscale->get_value();
    controllerDataOut.des_motor_ang_right_B   = position_right_B_hscale->get_value();
    controllerDataOut.des_motor_ang_right_hip = position_right_hip_hscale->get_value();
    controllerDataOut.leg_motor_p_gain        = position_leg_motor_p_gain_spinbutton->get_value();
    controllerDataOut.leg_motor_d_gain        = position_leg_motor_d_gain_spinbutton->get_value();
    controllerDataOut.hip_motor_p_gain        = position_hip_motor_p_gain_spinbutton->get_value();
    controllerDataOut.hip_motor_d_gain        = position_hip_motor_d_gain_spinbutton->get_value();

    pub.publish(controllerDataOut);
}

void guiTakedown() {
}

