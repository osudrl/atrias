/*
 * controller_gui.cpp
 *
 * Leg Angle Sine Wave Controller
 *
 *  Created on: May 6, 2012
 *      Author: Michael Anderson
 */

#include <ac_leg_angle_sine_wave/controller_gui.h>

ControllerInitResult guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	ControllerInitResult cir;
    cir.error = true;

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
        cir.error = false;
    }
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = 0;
    return cir;
}

void guiUpdate(robot_state state, ByteArray controllerStatus, ByteArray &controllerInput) {
    InputData out;

    out.leg_ang_frq = leg_angle_frequency_hscale->get_value();
    out.leg_ang_amp = leg_angle_amplitude_hscale->get_value();
    out.leg_len_frq = leg_length_frequency_hscale->get_value();
    out.leg_len_amp = leg_length_amplitude_hscale->get_value();
    out.p_gain = p_sine_wave_hscale->get_value();
    out.d_gain = d_sine_wave_hscale->get_value();

    structToByteArray(out, controllerInput);
}

void guiStandby(robot_state state) {

}

void guiTakedown() {

}
