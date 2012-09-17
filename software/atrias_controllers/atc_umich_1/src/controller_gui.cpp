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
    gui->get_widget("kp1_hscale", kp1_hscale);
    gui->get_widget("kp2_hscale", kp2_hscale);
    gui->get_widget("kp3_hscale", kp3_hscale);
    gui->get_widget("kd1_hscale", kd1_hscale);
    gui->get_widget("kd2_hscale", kd2_hscale);
    gui->get_widget("kd3_hscale", kd3_hscale);
    gui->get_widget("epsilon_spinbutton", epsilon_spinbutton);

    if (q1r_hscale && q2r_hscale && q3r_hscale &&
        q1l_hscale && q2l_hscale && q3l_hscale &&
        kp1_hscale && kp2_hscale && kp3_hscale &&
        kd1_hscale && kd2_hscale && kd3_hscale &&
        epsilon_spinbutton) {
        // Set ranges.
        q1r_hscale->set_range(140, 220);
        q2r_hscale->set_range(140, 220);
        q3r_hscale->set_range(-15, 15);
        q1l_hscale->set_range(140, 220);
        q2l_hscale->set_range(140, 220);
        q3l_hscale->set_range(-15, 15);
        kp1_hscale->set_range(   ,    );
        kp2_hscale->set_range(   ,    );
        kp3_hscale->set_range(   ,    );
        kd1_hscale->set_range(   ,    );
        kd2_hscale->set_range(   ,    );
        kd3_hscale->set_range(   ,    );
        epsilon_spinbutton->set_range(,);

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
    q3l_hscale->set_value(controllerDataOut.kp1);
    q3l_hscale->set_value(controllerDataOut.kp2);
    q3l_hscale->set_value(controllerDataOut.kp3);
    q3l_hscale->set_value(controllerDataOut.kd1);
    q3l_hscale->set_value(controllerDataOut.kd2);
    q3l_hscale->set_value(controllerDataOut.kd3);
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
    controllerDataOut.kp1 = kp1_hscale->get_value();
    controllerDataOut.kp2 = kp2_hscale->get_value();
    controllerDataOut.kp3 = kp3_hscale->get_value();
    controllerDataOut.kd1 = kd1_hscale->get_value();
    controllerDataOut.kd2 = kd2_hscale->get_value();
    controllerDataOut.kd3 = kd3_hscale->get_value();
    controllerDataOut.epsilon = epsilon_spinbutton->get_value();
    pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

