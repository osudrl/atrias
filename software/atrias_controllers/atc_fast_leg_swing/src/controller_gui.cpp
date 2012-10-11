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
    gui->get_widget("frequency_hscale", frequency_hscale);
    gui->get_widget("leg_magnitude_hscale", leg_magnitude_hscale);
    gui->get_widget("hip_magnitude_hscale", hip_magnitude_hscale);
    gui->get_widget("leg_p_spinbutton", leg_p_spinbutton);
    gui->get_widget("leg_d_spinbutton", leg_d_spinbutton);
    gui->get_widget("hip_p_spinbutton", hip_p_spinbutton);
    gui->get_widget("hip_d_spinbutton", hip_d_spinbutton);
    gui->get_widget("sweep_radiobutton", sweep_radiobutton);
    gui->get_widget("extend_radiobutton", extend_radiobutton);
    gui->get_widget("demo_enable_togglebutton", demo_enable_togglebutton);

    if (frequency_hscale && leg_magnitude_hscale && hip_magnitude_hscale &&
            leg_p_spinbutton && leg_d_spinbutton && hip_p_spinbutton && hip_d_spinbutton &&
            sweep_radiobutton && extend_radiobutton &&
            demo_enable_togglebutton) {
        // Set ranges.
        frequency_hscale->set_range(0.0, 0.7);
        leg_magnitude_hscale->set_range(0, 0.85);   // Magnitude is distance away from center.
        hip_magnitude_hscale->set_range(0, 0.15);
        leg_p_spinbutton->set_range(0, 2000);
        leg_d_spinbutton->set_range(0, 200);
        hip_p_spinbutton->set_range(0, 500);
        hip_d_spinbutton->set_range(0, 50);

        frequency_hscale->set_value(0.3);
        leg_magnitude_hscale->set_value(0.0);   // Magnitude is distance away from center.
        hip_magnitude_hscale->set_value(0.0);
        leg_p_spinbutton->set_value(600.0);
        leg_d_spinbutton->set_value(20.0);
        hip_p_spinbutton->set_value(150.0);
        hip_d_spinbutton->set_value(10.0);
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
    // Get parameters in the atrias_gui namespace.
    nh.getParam("/atrias_gui/frequency", controllerDataOut.frequency);
    nh.getParam("/atrias_gui/leg_magnitude", controllerDataOut.leg_magnitude);
    nh.getParam("/atrias_gui/hip_magnitude", controllerDataOut.hip_magnitude);
    nh.getParam("/atrias_gui/leg_p_gain", controllerDataOut.leg_p_gain);
    nh.getParam("/atrias_gui/leg_d_gain", controllerDataOut.leg_d_gain);
    nh.getParam("/atrias_gui/hip_p_gain", controllerDataOut.hip_p_gain);
    nh.getParam("/atrias_gui/hip_d_gain", controllerDataOut.hip_d_gain);

    // Configure the GUI.
    frequency_hscale->set_value(controllerDataOut.frequency);
    leg_magnitude_hscale->set_value(controllerDataOut.leg_magnitude);
    hip_magnitude_hscale->set_value(controllerDataOut.hip_magnitude);
    leg_p_spinbutton->set_value(controllerDataOut.leg_p_gain);
    leg_d_spinbutton->set_value(controllerDataOut.leg_d_gain);
    hip_p_spinbutton->set_value(controllerDataOut.hip_p_gain);
    hip_d_spinbutton->set_value(controllerDataOut.hip_d_gain);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    nh.setParam("/atrias_gui/frequency", controllerDataOut.frequency);
    nh.setParam("/atrias_gui/leg_magnitude", controllerDataOut.leg_magnitude);
    nh.setParam("/atrias_gui/hip_magnitude", controllerDataOut.hip_magnitude);
    nh.setParam("/atrias_gui/leg_p_gain", controllerDataOut.leg_p_gain);
    nh.setParam("/atrias_gui/leg_d_gain", controllerDataOut.leg_d_gain);
    nh.setParam("/atrias_gui/hip_p_gain", controllerDataOut.hip_p_gain);
    nh.setParam("/atrias_gui/hip_d_gain", controllerDataOut.hip_d_gain);
}

//! \brief Update the GUI.
void guiUpdate() {
    controllerDataOut.frequency = frequency_hscale->get_value();
    controllerDataOut.leg_magnitude = leg_magnitude_hscale->get_value();
    controllerDataOut.hip_magnitude = hip_magnitude_hscale->get_value();
    controllerDataOut.leg_p_gain = leg_p_spinbutton->get_value();
    controllerDataOut.leg_d_gain = leg_d_spinbutton->get_value();
    controllerDataOut.hip_p_gain = hip_p_spinbutton->get_value();
    controllerDataOut.hip_d_gain = hip_d_spinbutton->get_value();
    controllerDataOut.mode = (sweep_radiobutton->get_active()) ? 0 : 1;
    controllerDataOut.demoEnabled = (demo_enable_togglebutton->get_active()) ? 1 : 0;

    if (sweep_radiobutton->get_active()) {   // Sweep mode
        leg_magnitude_hscale->set_range(0, 0.85);
    }
    else {   // Extend mode.
        leg_magnitude_hscale->set_range(0, 0.425);
        if (leg_magnitude_hscale->get_value() > 0.425) {
            leg_magnitude_hscale->set_value(0.425);
        }
    }

    pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

