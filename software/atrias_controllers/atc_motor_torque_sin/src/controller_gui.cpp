/*
 * controller_gui.cpp
 *
 * Leg Angle Sine Wave Controller
 *
 *  Created on: May 6, 2012
 *      Author: Michael Anderson
 */

#include <atc_motor_torque_sin/controller_gui.h>

void reset_gui() {
    motor_offset_spinbutton->set_value(0.0);
    amplitude_hscale->set_value(0.0);
    frequency_hscale->set_value(0.0);
}

bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    gui->get_widget("motor_selection_combobox", motor_selection_combobox);
    gui->get_widget("motor_offset_spinbutton", motor_offset_spinbutton);
    gui->get_widget("amplitude_hscale", amplitude_hscale);
    gui->get_widget("frequency_hscale", frequency_hscale);

    if (motor_selection_combobox && motor_offset_spinbutton &&
            amplitude_hscale && frequency_hscale) {
        motor_offset_spinbutton->set_range(-2.0, 2.0);
        amplitude_hscale->set_range(0., 10.0);
        frequency_hscale->set_range(0., 20.);
        pub = nh.advertise<atc_motor_torque_sin::controller_input>("atc_motor_torque_sin_input", 0);

        // Reset the GUI when motor selection is changed.
        motor_selection_combobox->signal_changed().connect(sigc::ptr_fun((void(*)()) reset_gui));

        return true;
    }
    return false;
}

void guiUpdate() {
    controllerDataOut.motor  = motor_selection_combobox->get_active_row_number();
    controllerDataOut.offset = motor_offset_spinbutton->get_value();
    controllerDataOut.frq    = frequency_hscale->get_value();
    controllerDataOut.amp    = amplitude_hscale->get_value();
    pub.publish(controllerDataOut);
}

void guiTakedown() {
}
