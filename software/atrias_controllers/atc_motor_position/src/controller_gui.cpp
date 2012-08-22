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
    gui->get_widget("position_A_hscale", position_A_hscale);
    gui->get_widget("position_B_hscale", position_B_hscale);
    gui->get_widget("position_p_hscale", p_hscale);
    gui->get_widget("position_d_hscale", d_hscale);
    gui->get_widget("set_position_checkbutton", set_position_checkbutton);

    if (position_A_hscale && position_B_hscale && p_hscale && d_hscale &&
            set_position_checkbutton) {
        // Set ranges.
        position_A_hscale->set_range(-5, 5);
        position_B_hscale->set_range(-5, 5);
        p_hscale->set_range(0., 50000.);
        d_hscale->set_range(0., 5000.);

        // Set parameters in the atrias_gui namespace.
        nh.param("/atrias_gui/a_position",            a_position_param, 0.);
        nh.param("/atrias_gui/b_position",            b_position_param, 0.);
        nh.param("/atrias_gui/motor_position_p_gain", p_gain_param,     0.);
        nh.param("/atrias_gui/motor_position_d_gain", d_gain_param,     0.);

        // Set values.
        position_A_hscale->set_value(a_position_param);
        position_B_hscale->set_value(b_position_param);
        p_hscale->set_value(p_gain_param);
        d_hscale->set_value(d_gain_param);

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

void guiUpdate() {
    controllerDataOut.des_motor_ang_A = position_A_hscale->get_value();
    controllerDataOut.des_motor_ang_B = position_B_hscale->get_value();
    controllerDataOut.p_gain = p_hscale->get_value();
    controllerDataOut.d_gain = d_hscale->get_value();
    pub.publish(controllerDataOut);
}

void guiTakedown() {
}

