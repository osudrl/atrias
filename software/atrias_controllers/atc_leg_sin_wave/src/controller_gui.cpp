/*
 * controller_gui.cpp
 *
 * Leg Angle Sine Wave Controller
 *
 *  Created on: May 6, 2012
 *      Author: Michael Anderson
 */

#include <atc_leg_sin_wave/controller_gui.h>

bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    gui->get_widget("angle_amplitude_hscale", leg_angle_amplitude_hscale);
    gui->get_widget("angle_frequency_hscale", leg_angle_frequency_hscale);
    gui->get_widget("length_amplitude_hscale", leg_length_amplitude_hscale);
    gui->get_widget("length_frequency_hscale", leg_length_frequency_hscale);
    gui->get_widget("leg_p_gain_entry", leg_p_gain_entry);
    gui->get_widget("leg_d_gain_entry", leg_d_gain_entry);
    gui->get_widget("hip_angle_amplitude_hscale", hip_angle_amplitude_hscale);
    gui->get_widget("hip_angle_frequency_hscale", hip_angle_frequency_hscale);
    gui->get_widget("hip_p_gain_entry", hip_p_gain_entry);
    gui->get_widget("hip_d_gain_entry", hip_d_gain_entry);

    printf("%p, %p, %p, %p, %p, %p, %p, %p, %p, %p\n",
leg_angle_amplitude_hscale, leg_angle_frequency_hscale, leg_length_amplitude_hscale, leg_length_frequency_hscale, leg_p_gain_entry, leg_d_gain_entry, hip_angle_amplitude_hscale, hip_angle_frequency_hscale, hip_p_gain_entry, hip_d_gain_entry);

    if (leg_angle_amplitude_hscale && leg_angle_frequency_hscale && leg_length_amplitude_hscale
            && leg_length_frequency_hscale && hip_angle_amplitude_hscale && hip_angle_frequency_hscale
            && leg_p_gain_entry && leg_d_gain_entry && hip_p_gain_entry && hip_d_gain_entry) {
        // Set ranges.
        leg_angle_amplitude_hscale->set_range(0., 1.0);
        leg_angle_frequency_hscale->set_range(0., 20.);
        leg_length_amplitude_hscale->set_range(0., 0.2);
        leg_length_frequency_hscale->set_range(0., 20.);
        leg_p_gain_entry->set_range(0., 10000.);
        leg_d_gain_entry->set_range(0., 500.);

        hip_angle_amplitude_hscale->set_range(0., 0.174532);
        hip_angle_frequency_hscale->set_range(0., 10.);
        hip_p_gain_entry->set_range(0., 300.);
        hip_d_gain_entry->set_range(0., 50.);

        // Set up subscriber and publisher.
        sub = nh.subscribe("atc_leg_sin_wave_status", 0, controllerCallback);
        pub = nh.advertise<atc_leg_sin_wave::controller_input>("atc_leg_sin_wave_input", 0);
        return true;
    }
    return false;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_leg_sin_wave::controller_status &status) {
    controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
    // Get parameters in the atrias_gui namespace.
    nh.getParam("/atrias_gui/leg_angle_amplitude",  leg_angle_amplitude_param);
    nh.getParam("/atrias_gui/leg_angle_frequency",  leg_angle_frequency_param);
    nh.getParam("/atrias_gui/leg_length_amplitude", leg_length_amplitude_param);
    nh.getParam("/atrias_gui/leg_length_frequency", leg_length_frequency_param);
    nh.getParam("/atrias_gui/p_gain",               p_gain_param);
    nh.getParam("/atrias_gui/d_gain",               d_gain_param);

    // Configure the GUI.
    leg_angle_amplitude_hscale->set_value(leg_angle_amplitude_param);
    leg_angle_frequency_hscale->set_value(leg_angle_frequency_param);
    leg_length_amplitude_hscale->set_value(leg_length_amplitude_param);
    leg_length_frequency_hscale->set_value(leg_length_frequency_param);
    leg_p_gain_entry->set_value(p_gain_param);
    leg_d_gain_entry->set_value(d_gain_param);
}
//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    nh.setParam("/atrias_gui/leg_angle_amplitude",  leg_angle_amplitude_param);
    nh.setParam("/atrias_gui/leg_angle_frequency",  leg_angle_frequency_param);
    nh.setParam("/atrias_gui/leg_length_amplitude", leg_length_amplitude_param);
    nh.setParam("/atrias_gui/leg_length_frequency", leg_length_frequency_param);
    nh.getParam("/atrias_gui/p_gain",               p_gain_param);
    nh.getParam("/atrias_gui/d_gain",               d_gain_param);
}


void guiUpdate() {
    controllerDataOut.leg_ang_amp = leg_angle_amplitude_param  = leg_angle_amplitude_hscale->get_value();
    controllerDataOut.leg_ang_frq = leg_angle_frequency_param  = leg_angle_frequency_hscale->get_value();
    controllerDataOut.leg_len_amp = leg_length_amplitude_param = leg_length_amplitude_hscale->get_value();
    controllerDataOut.leg_len_frq = leg_length_frequency_param = leg_length_frequency_hscale->get_value();
    controllerDataOut.p_gain      = p_gain_param               = leg_p_gain_entry->get_value();
    controllerDataOut.d_gain      = d_gain_param               = leg_d_gain_entry->get_value();
    controllerDataOut.hip_amp     = hip_angle_amplitude_hscale->get_value();
    controllerDataOut.hip_frq     = hip_angle_frequency_hscale->get_value();
    controllerDataOut.hip_p_gain  = hip_p_gain_entry->get_value();
    controllerDataOut.hip_d_gain  = hip_d_gain_entry->get_value();
    pub.publish(controllerDataOut);
}

void guiTakedown() {
}
