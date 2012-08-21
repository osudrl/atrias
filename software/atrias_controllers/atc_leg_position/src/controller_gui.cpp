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

        // Set parameters.
        nh.param("leg_length",          leg_length_param, 0.6);
        nh.param("leg_angle",           leg_angle_param,  1.5);
        nh.param("leg_position_p_gain", leg_p_gain_param, 0.);
        nh.param("leg_position_d_gain", leg_d_gain_param, 0.);
        nh.param("hip_angle",           hip_angle_param,  0.);
        nh.param("hip_position_p_gain", hip_p_gain_param, 0.);
        nh.param("hip_position_d_gain", hip_d_gain_param, 0.);

        // Set values.
        leg_length_hscale->set_value(leg_length_param);
        leg_angle_hscale->set_value(leg_angle_param);
        p_leg_position_hscale->set_value(leg_p_gain_param);
        d_leg_position_hscale->set_value(leg_d_gain_param);
        hip_position_ang->set_value(hip_angle_param);
        hip_position_p->set_value(hip_p_gain_param);
        hip_position_d->set_value(hip_d_gain_param);

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

void guiUpdate() {
    controllerDataOut.leg_ang = leg_angle_hscale->get_value();
    controllerDataOut.leg_len = leg_length_hscale->get_value();
    controllerDataOut.hip_ang = hip_position_ang->get_value();
    controllerDataOut.p_gain = p_leg_position_hscale->get_value();
    controllerDataOut.d_gain = d_leg_position_hscale->get_value();
    controllerDataOut.hip_p_gain = hip_position_p->get_value();
    controllerDataOut.hip_d_gain = hip_position_d->get_value();
    pub.publish(controllerDataOut);
}

void guiTakedown() {
}
