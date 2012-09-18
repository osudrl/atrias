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
    gui->get_widget("kp1_spinbutton", kp1_spinbutton);
    gui->get_widget("kp2_spinbutton", kp2_spinbutton);
    gui->get_widget("kp3_spinbutton", kp3_spinbutton);
    gui->get_widget("kd1_spinbutton", kd1_spinbutton);
    gui->get_widget("kd2_spinbutton", kd2_spinbutton);
    gui->get_widget("kd3_spinbutton", kd3_spinbutton);
    gui->get_widget("epsilon_spinbutton", epsilon_spinbutton);

    if (q1r_hscale && q2r_hscale && q3r_hscale &&
        q1l_hscale && q2l_hscale && q3l_hscale &&
        kp1_spinbutton && kp2_spinbutton && kp3_spinbutton &&
        kd1_spinbutton && kd2_spinbutton && kd3_spinbutton &&
        epsilon_spinbutton) {
        // Set ranges.
        q1r_hscale->set_range(140, 220);
        q2r_hscale->set_range(140, 220);
        q3r_hscale->set_range(-15, 15);
        q1l_hscale->set_range(140, 220);
        q2l_hscale->set_range(140, 220);
        q3l_hscale->set_range(-15, 15);
        kp1_spinbutton->set_range(0, 50);
        kp2_spinbutton->set_range(0, 50);
        kp3_spinbutton->set_range(0, 50);
        kd1_spinbutton->set_range(0, 10);
        kd2_spinbutton->set_range(0, 10);
        kd3_spinbutton->set_range(0, 10);
        epsilon_spinbutton->set_range(0, 1);

        // Set up subscriber and publisher.
        pub = nh.advertise<atc_umich_1::controller_input>("atc_umich_1_input", 0);
        return true;
    }
    return false;
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
    nh.getParam("/atrias_gui/kp1", controllerDataOut.kp1);
    nh.getParam("/atrias_gui/kp2", controllerDataOut.kp2);
    nh.getParam("/atrias_gui/kp3", controllerDataOut.kp3);
    nh.getParam("/atrias_gui/kd1", controllerDataOut.kd1);
    nh.getParam("/atrias_gui/kd2", controllerDataOut.kd2);
    nh.getParam("/atrias_gui/kd3", controllerDataOut.kd3);
    nh.getParam("/atrias_gui/epsilon", controllerDataOut.epsilon);

    // Configure the GUI.
    q1r_hscale->set_value(controllerDataOut.q1r);
    q2r_hscale->set_value(controllerDataOut.q2r);
    q3r_hscale->set_value(controllerDataOut.q3r);
    q1l_hscale->set_value(controllerDataOut.q1l);
    q2l_hscale->set_value(controllerDataOut.q2l);
    q3l_hscale->set_value(controllerDataOut.q3l);
    kp1_spinbutton->set_value(controllerDataOut.kp1);
    kp2_spinbutton->set_value(controllerDataOut.kp2);
    kp3_spinbutton->set_value(controllerDataOut.kp3);
    kd1_spinbutton->set_value(controllerDataOut.kd1);
    kd2_spinbutton->set_value(controllerDataOut.kd2);
    kd3_spinbutton->set_value(controllerDataOut.kd3);
    epsilon_spinbutton->set_value(controllerDataOut.epsilon);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    nh.setParam("/atrias_gui/q1r", controllerDataOut.q1r);
    nh.setParam("/atrias_gui/q2r", controllerDataOut.q2r);
    nh.setParam("/atrias_gui/q3r", controllerDataOut.q3r);
    nh.setParam("/atrias_gui/q1l", controllerDataOut.q1l);
    nh.setParam("/atrias_gui/q2l", controllerDataOut.q2l);
    nh.setParam("/atrias_gui/q3l", controllerDataOut.q3l);
    nh.setParam("/atrias_gui/kp1", controllerDataOut.kp1);
    nh.setParam("/atrias_gui/kp2", controllerDataOut.kp2);
    nh.setParam("/atrias_gui/kp3", controllerDataOut.kp3);
    nh.setParam("/atrias_gui/kd1", controllerDataOut.kd1);
    nh.setParam("/atrias_gui/kd2", controllerDataOut.kd2);
    nh.setParam("/atrias_gui/kd3", controllerDataOut.kd3);
    nh.setParam("/atrias_gui/epsilon", controllerDataOut.epsilon);
}

//! \brief Update the GUI.
void guiUpdate() {
    controllerDataOut.q1r = q1r_hscale->get_value();
    controllerDataOut.q2r = q2r_hscale->get_value();
    controllerDataOut.q3r = q3r_hscale->get_value();
    controllerDataOut.q1l = q1l_hscale->get_value();
    controllerDataOut.q2l = q2l_hscale->get_value();
    controllerDataOut.q3l = q3l_hscale->get_value();
    controllerDataOut.kp1 = kp1_spinbutton->get_value();
    controllerDataOut.kp2 = kp2_spinbutton->get_value();
    controllerDataOut.kp3 = kp3_spinbutton->get_value();
    controllerDataOut.kd1 = kd1_spinbutton->get_value();
    controllerDataOut.kd2 = kd2_spinbutton->get_value();
    controllerDataOut.kd3 = kd3_spinbutton->get_value();
    controllerDataOut.epsilon = epsilon_spinbutton->get_value();
    pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

