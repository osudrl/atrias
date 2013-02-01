/*
 * controller_gui.cpp
 *
 * atc_single_leg_hopping controller
 *
 *  Created on: Jan 31, 2013
 *      Author: Mikhail Jones
 */

#include <atc_single_leg_hopping/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    gui->get_widget("flight_leg_P_gain_spinbutton", flight_leg_P_gain_spinbutton);
    gui->get_widget("flight_leg_D_gain_spinbutton", flight_leg_D_gain_spinbutton);
    gui->get_widget("flight_hip_P_gain_spinbutton", flight_hip_P_gain_spinbutton);
    gui->get_widget("flight_hip_D_gain_spinbutton", flight_hip_D_gain_spinbutton);
    gui->get_widget("stance_leg_P_gain_spinbutton", stance_leg_P_gain_spinbutton);
    gui->get_widget("stance_leg_D_gain_spinbutton", stance_leg_D_gain_spinbutton);
    gui->get_widget("stance_hip_P_gain_spinbutton", stance_hip_P_gain_spinbutton);
    gui->get_widget("stance_hip_D_gain_spinbutton", stance_hip_D_gain_spinbutton);

    if (flight_leg_P_gain_spinbutton && flight_leg_D_gain_spinbutton) {

        // Set ranges.
        flight_leg_P_gain_spinbutton->set_range(0., 1000.);
        flight_leg_D_gain_spinbutton->set_range(0., 100.);
        flight_hip_P_gain_spinbutton->set_range(0., 1000.);
        flight_hip_D_gain_spinbutton->set_range(0., 100.);
        stance_leg_P_gain_spinbutton->set_range(0., 1000.);
        stance_leg_D_gain_spinbutton->set_range(0., 100.);
        stance_hip_P_gain_spinbutton->set_range(0., 1000.);
        stance_hip_D_gain_spinbutton->set_range(0., 100.);

        // Set default values
        flight_leg_P_gain_spinbutton->set_value(10.);
        flight_leg_D_gain_spinbutton->set_value(10.);
        flight_hip_P_gain_spinbutton->set_value(10.);
        flight_hip_D_gain_spinbutton->set_value(10.);
        stance_leg_P_gain_spinbutton->set_value(10.);
        stance_leg_D_gain_spinbutton->set_value(10.);
        stance_hip_P_gain_spinbutton->set_value(10.);
        stance_hip_D_gain_spinbutton->set_value(10.);


        // Set up subscriber and publisher.
        sub = nh.subscribe("atc_single_leg_hopping_status", 0, controllerCallback);
        pub = nh.advertise<atc_single_leg_hopping::controller_input>("atc_single_leg_hopping_input", 0);
        return true;
    }
    return false;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_single_leg_hopping::controller_status &status) {
    controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
    // Get parameters in the atrias_gui namespace.
    nh.getParam("/atrias_gui/flight_leg_P_gain", controllerDataOut.flight_leg_P_gain);
    nh.getParam("/atrias_gui/flight_leg_D_gain", controllerDataOut.flight_leg_D_gain);
    nh.getParam("/atrias_gui/flight_hip_P_gain", controllerDataOut.flight_hip_P_gain);
    nh.getParam("/atrias_gui/flight_hip_D_gain", controllerDataOut.flight_hip_D_gain);
    nh.getParam("/atrias_gui/stance_leg_P_gain", controllerDataOut.stance_leg_P_gain);
    nh.getParam("/atrias_gui/stance_leg_D_gain", controllerDataOut.stance_leg_D_gain);
    nh.getParam("/atrias_gui/stance_hip_P_gain", controllerDataOut.stance_hip_P_gain);
    nh.getParam("/atrias_gui/stance_hip_D_gain", controllerDataOut.stance_hip_D_gain);

    // Configure the GUI.
    flight_leg_P_gain_spinbutton->set_value(controllerDataOut.flight_leg_P_gain);
    flight_leg_D_gain_spinbutton->set_value(controllerDataOut.flight_leg_D_gain);
    flight_hip_P_gain_spinbutton->set_value(controllerDataOut.flight_hip_P_gain);
    flight_hip_D_gain_spinbutton->set_value(controllerDataOut.flight_hip_D_gain);
    stance_leg_P_gain_spinbutton->set_value(controllerDataOut.stance_leg_P_gain);
    stance_leg_D_gain_spinbutton->set_value(controllerDataOut.stance_leg_D_gain);
    stance_hip_P_gain_spinbutton->set_value(controllerDataOut.stance_hip_P_gain);
    stance_hip_D_gain_spinbutton->set_value(controllerDataOut.stance_hip_D_gain);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    nh.setParam("/atrias_gui/flight_leg_P_gain", controllerDataOut.flight_leg_P_gain);
    nh.setParam("/atrias_gui/flight_leg_D_gain", controllerDataOut.flight_leg_D_gain);
    nh.setParam("/atrias_gui/flight_hip_P_gain", controllerDataOut.flight_hip_P_gain);
    nh.setParam("/atrias_gui/flight_hip_D_gain", controllerDataOut.flight_hip_D_gain);
    nh.setParam("/atrias_gui/stance_leg_P_gain", controllerDataOut.stance_leg_P_gain);
    nh.setParam("/atrias_gui/stance_leg_D_gain", controllerDataOut.stance_leg_D_gain);
    nh.setParam("/atrias_gui/stance_hip_P_gain", controllerDataOut.stance_hip_P_gain);
    nh.setParam("/atrias_gui/stance_hip_D_gain", controllerDataOut.stance_hip_D_gain);
}

//! \brief Update the GUI.
void guiUpdate() {
    controllerDataOut.flight_leg_P_gain = flight_leg_P_gain_spinbutton->get_value();
    controllerDataOut.flight_leg_D_gain = flight_leg_D_gain_spinbutton->get_value();
    controllerDataOut.flight_hip_P_gain = flight_hip_P_gain_spinbutton->get_value();
    controllerDataOut.flight_hip_D_gain = flight_hip_D_gain_spinbutton->get_value();
    controllerDataOut.stance_leg_P_gain = stance_leg_P_gain_spinbutton->get_value();
    controllerDataOut.stance_leg_D_gain = stance_leg_D_gain_spinbutton->get_value();
    controllerDataOut.stance_hip_P_gain = stance_hip_P_gain_spinbutton->get_value();
    controllerDataOut.stance_hip_D_gain = stance_hip_D_gain_spinbutton->get_value();
    pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

