/*
 * controller_gui.cpp
 *
 * Leg Angle Sine Wave Controller
 *
 *  Created on: May 6, 2012
 *      Author: Michael Anderson
 */

#include <atc_motor_torque_sin/controller_gui.h>

bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    gui->get_widget("motor_selection_combobox", motor_selection_combobox);
    gui->get_widget("motor_offset_spinbutton", motor_offset_spinbutton);
    gui->get_widget("angle_amplitude_hscale", angle_amplitude_hscale);
    gui->get_widget("angle_frequency_hscale", angle_frequency_hscale);
    gui->get_widget("length_amplitude_hscale", length_amplitude_hscale);
    gui->get_widget("length_frequency_hscale", length_frequency_hscale);

    if (motor_selection_combobox && motor_offset_spinbutton &&
            leg_angle_amplitude_hscale && leg_angle_frequency_hscale &&
            leg_length_amplitude_hscale && leg_length_frequency_hscale) {
        motor_offset_spinbutton->set_range(-0.1, 0.1);
        leg_angle_amplitude_hscale->set_range(0., 1.0);
        leg_angle_frequency_hscale->set_range(0., 20.);
        leg_length_amplitude_hscale->set_range(0., 0.2);
        leg_length_frequency_hscale->set_range(0., 20.);
        pub = nh.advertise<atc_motor_torque_sin::controller_input>("atc_motor_torque_sin_input", 0);
        return true;
    }
    return false;
}

void guiUpdate() {
    controllerDataOut.motor_selection = motor_selection_combobox->gtk_combo_box_get_active();
    controllerDataOut.motor_offset    = motor_offset_spinbutton->get_value();
    controllerDataOut.leg_ang_frq = leg_angle_frequency_hscale->get_value();
    controllerDataOut.leg_ang_amp = leg_angle_amplitude_hscale->get_value();
    controllerDataOut.leg_len_frq = leg_length_frequency_hscale->get_value();
    controllerDataOut.leg_len_amp = leg_length_amplitude_hscale->get_value();
    pub.publish(controllerDataOut);
}

void guiTakedown() {
}
