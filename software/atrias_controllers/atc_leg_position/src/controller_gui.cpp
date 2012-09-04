/*
 * controller_gui.cpp
 *
 * Leg Position Controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <atc_leg_position/controller_gui.h>

bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    gui->get_widget("length_hscale", leg_length_hscale);
    gui->get_widget("angle_hscale", leg_angle_hscale);
    gui->get_widget("p_hscale", p_leg_position_hscale);
    gui->get_widget("d_hscale", d_leg_position_hscale);
    gui->get_widget("hip_position_ang", hip_position_ang);
    gui->get_widget("hip_position_p", hip_position_p);
    gui->get_widget("hip_position_d", hip_position_d);
    gui->get_widget("update_checkbutton", update_checkbutton);

    if (p_leg_position_hscale && d_leg_position_hscale && hip_position_ang &&
            hip_position_p && hip_position_d && leg_length_hscale &&
            leg_angle_hscale && update_checkbutton) {
        // Set ranges.
        leg_length_hscale->set_range(0.5, 1.);
        leg_angle_hscale->set_range(0.29, 2.85);
        p_leg_position_hscale->set_range(0., 10000.);
        d_leg_position_hscale->set_range(0., 300.);
        hip_position_ang->set_range(-0.209, 0.209);
        hip_position_p->set_range(0., 10000.);
        hip_position_d->set_range(0., 300.);

        // Set up subscriber and publisher.
        sub = nh.subscribe("atc_leg_position_status", 0, controllerCallback);
        pub = nh.advertise<atc_leg_position::controller_input>("atc_leg_position_input", 0);

        return true;
    }

    return false;
}

void controllerCallback(const atc_leg_position::controller_status &status) {
    controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
    // Get parameters in the atrias_gui namespace.
    nh.getParam("/atrias_gui/leg_length",          controllerDataOut.leg_len);
    nh.getParam("/atrias_gui/leg_angle",           controllerDataOut.leg_ang);
    nh.getParam("/atrias_gui/leg_position_p_gain", controllerDataOut.p_gain);
    nh.getParam("/atrias_gui/leg_position_d_gain", controllerDataOut.d_gain);
    nh.getParam("/atrias_gui/hip_angle",           controllerDataOut.hip_ang);
    nh.getParam("/atrias_gui/hip_position_p_gain", controllerDataOut.hip_p_gain);
    nh.getParam("/atrias_gui/hip_position_d_gain", controllerDataOut.hip_d_gain);

    // Configure the GUI.
    leg_length_hscale->set_value(controllerDataOut.leg_len);
    leg_angle_hscale->set_value(controllerDataOut.leg_ang);
    p_leg_position_hscale->set_value(controllerDataOut.p_gain);
    d_leg_position_hscale->set_value(controllerDataOut.d_gain);
    hip_position_ang->set_value(controllerDataOut.hip_ang);
    hip_position_p->set_value(controllerDataOut.hip_p_gain);
    hip_position_d->set_value(controllerDataOut.hip_d_gain);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    nh.setParam("/atrias_gui/leg_length",          controllerDataOut.leg_len);
    nh.setParam("/atrias_gui/leg_angle",           controllerDataOut.leg_ang);
    nh.setParam("/atrias_gui/leg_position_p_gain", controllerDataOut.p_gain);
    nh.setParam("/atrias_gui/leg_position_d_gain", controllerDataOut.d_gain);
    nh.setParam("/atrias_gui/hip_angle",           controllerDataOut.hip_ang);
    nh.setParam("/atrias_gui/hip_position_p_gain", controllerDataOut.hip_p_gain);
    nh.setParam("/atrias_gui/hip_position_d_gain", controllerDataOut.hip_d_gain);
}

//! \brief Update the GUI.
void guiUpdate() {
    controllerDataOut.leg_len    = leg_length_hscale->get_value();
    controllerDataOut.leg_ang    = leg_angle_hscale->get_value();
    controllerDataOut.p_gain     = p_leg_position_hscale->get_value();
    controllerDataOut.d_gain     = d_leg_position_hscale->get_value();
    controllerDataOut.hip_ang    = hip_position_ang->get_value();
    controllerDataOut.hip_p_gain = hip_position_p->get_value();
    controllerDataOut.hip_d_gain = hip_position_d->get_value();
    pub.publish(controllerDataOut);
}

void guiTakedown() {
}
