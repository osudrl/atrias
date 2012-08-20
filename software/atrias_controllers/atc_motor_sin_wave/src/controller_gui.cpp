/*
 * controller_gui.cpp
 *
 * Leg Angle Sine Wave Controller
 *
 *  Created on: May 6, 2012
 *      Author: Michael Anderson
 */

#include <atc_motor_sin_wave/controller_gui.h>

bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    gui->get_widget("angle_amplitude_hscale", leg_angle_amplitude_hscale);
    gui->get_widget("angle_frequency_hscale", leg_angle_frequency_hscale);
    gui->get_widget("length_amplitude_hscale", leg_length_amplitude_hscale);
    gui->get_widget("length_frequency_hscale", leg_length_frequency_hscale);
    gui->get_widget("p_hscale", p_sine_wave_hscale);
    gui->get_widget("d_hscale", d_sine_wave_hscale);

    if (leg_angle_amplitude_hscale && leg_angle_frequency_hscale && leg_length_amplitude_hscale
            && leg_length_frequency_hscale && p_sine_wave_hscale && d_sine_wave_hscale) {
        leg_angle_amplitude_hscale->set_range(0., 1.0);
        leg_angle_frequency_hscale->set_range(0., 20.);
        leg_length_amplitude_hscale->set_range(0., 0.2);
        leg_length_frequency_hscale->set_range(0., 20.);
        p_sine_wave_hscale->set_range(0., 10000.);
        d_sine_wave_hscale->set_range(0., 100.);
        pub = nh.advertise<atc_motor_sin_wave::controller_input>("atc_motor_sin_wave_input", 0);
        return true;
    }
    return false;
}

void guiUpdate() {
    controllerDataOut.leg_ang_frq = leg_angle_frequency_hscale->get_value();
    controllerDataOut.leg_ang_amp = leg_angle_amplitude_hscale->get_value();
    controllerDataOut.leg_len_frq = leg_length_frequency_hscale->get_value();
    controllerDataOut.leg_len_amp = leg_length_amplitude_hscale->get_value();
    controllerDataOut.p_gain = p_sine_wave_hscale->get_value();
    controllerDataOut.d_gain = d_sine_wave_hscale->get_value();
    pub.publish(controllerDataOut);
}

void guiTakedown() {
}
