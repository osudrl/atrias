/*
 * controller_gui.cpp
 *
 * atc_umich_1 controller
 *
 *  Created on: September 17, 2012
 *      Author: Soo-Hyun Yoo
 */

#include <atc_umich_1/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    gui->get_widget("q1r_hscale", q1r_hscale);
    gui->get_widget("q2r_hscale", q2r_hscale);
    gui->get_widget("q3r_hscale", q3r_hscale);
    gui->get_widget("q1l_hscale", q1l_hscale);
    gui->get_widget("q2l_hscale", q2l_hscale);
    gui->get_widget("q3l_hscale", q3l_hscale);

    if (q1r_hscale && q2r_hscale && q3r_hscale &&
	q1l_hscale && q2l_hscale && q3l_hscale) {
        // Set ranges.
        q1r_hscale->set_range(140, 220);
        q2r_hscale->set_range(140, 220);
        q3r_hscale->set_range(-15, 15);
        q1l_hscale->set_range(140, 220);
        q2l_hscale->set_range(140, 220);
        q3l_hscale->set_range(-15, 15);

        // Set up subscriber and publisher.
        sub = nh.subscribe("atc_umich_1_status", 0, controllerCallback);
        pub = nh.advertise<atc_umich_1::controller_input>("atc_umich_1_input", 0);
        return true;
    }
    return false;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_umich_1::controller_status &status) {
    controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
    // Get parameters in the atrias_gui namespace.
    nh.getParam("/atrias_gui/q1r", controllerDataOut.q1r);
    nh.getParam("/atrias_gui/q2r", controllerDataOut.q2r);
    nh.getParam("/atrias_gui/q3r", controllerDataOut.q3r);
    nh.getParam("/atrias_gui/q1l", controllerDataOut.q1l);
    nh.getParam("/atrias_gui/q2l", controllerDataOut.q2l);
    nh.getParam("/atrias_gui/q3l", controllerDataOut.q3l);

    // Configure the GUI.
    q1r_hscale->set_value(controllerDataOut.q1r);
    q2r_hscale->set_value(controllerDataOut.q2r);
    q3r_hscale->set_value(controllerDataOut.q3r);
    q1l_hscale->set_value(controllerDataOut.q1l);
    q2l_hscale->set_value(controllerDataOut.q2l);
    q3l_hscale->set_value(controllerDataOut.q3l);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    nh.setParam("/atrias_gui/q1r", controllerDataOut.q1r);
    nh.setParam("/atrias_gui/q2r", controllerDataOut.q2r);
    nh.setParam("/atrias_gui/q3r", controllerDataOut.q3r);
    nh.setParam("/atrias_gui/q1l", controllerDataOut.q1l);
    nh.setParam("/atrias_gui/q2l", controllerDataOut.q2l);
    nh.setParam("/atrias_gui/q3l", controllerDataOut.q3l);
}

//! \brief Update the GUI.
void guiUpdate() {
    controllerDataOut.q1r = q1r_hscale->get_value();
    controllerDataOut.q2r = q2r_hscale->get_value();
    controllerDataOut.q3r = q3r_hscale->get_value();
    controllerDataOut.q1l = q1l_hscale->get_value();
    controllerDataOut.q2l = q2l_hscale->get_value();
    controllerDataOut.q3l = q3l_hscale->get_value();
    pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

