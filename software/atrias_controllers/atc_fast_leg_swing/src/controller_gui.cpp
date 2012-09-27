/*
 * controller_gui.cpp
 *
 * atc_fast_leg_swing controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <atc_fast_leg_swing/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    gui->get_widget("frequency_spinbutton", frequency_spinbutton);
    gui->get_widget("leg_magnitude_spinbutton", leg_magnitude_spinbutton);
    gui->get_widget("hip_magnitude_spinbutton", hip_magnitude_spinbutton);
    gui->get_widget("leg_p_spinbutton", leg_p_spinbutton);
    gui->get_widget("leg_d_spinbutton", leg_d_spinbutton);
    gui->get_widget("hip_p_spinbutton", hip_p_spinbutton);
    gui->get_widget("hip_d_spinbutton", hip_d_spinbutton);
    gui->get_widget("sweep_radiobutton", sweep_radiobutton);
    gui->get_widget("extend_radiobutton", extend_radiobutton);

    if (frequency_spinbutton && leg_magnitude_spinbutton && hip_magnitude_spinbutton &&
            leg_p_spinbutton && leg_d_spinbutton && hip_p_spinbutton && hip_d_spinbutton &&
            sweep_radiobutton && extend_radiobutton) {
        // Set ranges.
        frequency_spinbutton->set_range(0, 10);
        leg_magnitude_spinbutton->set_range(0, 0.85);   // Magnitude is distance away from center.
        hip_magnitude_spinbutton->set_range(0, 0.085);
        leg_p_spinbutton->set_range(0, 2000);
        leg_d_spinbutton->set_range(0, 200);
        hip_p_spinbutton->set_range(0, 500);
        hip_d_spinbutton->set_range(0, 50);

        // Set up subscriber and publisher.
        sub = nh.subscribe("atc_fast_leg_swing_status", 0, controllerCallback);
        pub = nh.advertise<atc_fast_leg_swing::controller_input>("atc_fast_leg_swing_input", 0);
        return true;
    }
    return false;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_fast_leg_swing::controller_status &status) {
    controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
    // Disabled.
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    // Disabled.
}

//! \brief Update the GUI.
void guiUpdate() {
    controllerDataOut.frequency = frequency_spinbutton->get_value();
    controllerDataOut.leg_magnitude = leg_magnitude_spinbutton->get_value();
    controllerDataOut.hip_magnitude = hip_magnitude_spinbutton->get_value();
    controllerDataOut.leg_p_gain = leg_p_spinbutton->get_value();
    controllerDataOut.leg_d_gain = leg_d_spinbutton->get_value();
    controllerDataOut.hip_p_gain = hip_p_spinbutton->get_value();
    controllerDataOut.hip_d_gain = hip_d_spinbutton->get_value();
    controllerDataOut.mode = (sweep_radiobutton->get_active()) ? 0 : 1;

    if (sweep_radiobutton->get_active()) {   // Sweep mode
        leg_magnitude_spinbutton->set_range(0, 0.85);
    }
    else {   // Extend mode.
        leg_magnitude_spinbutton->set_range(0, 0.25);
        if (leg_magnitude_spinbutton->get_value() > 0.25) {
            leg_magnitude_spinbutton->set_value(0.25);
        }
    }

    pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

