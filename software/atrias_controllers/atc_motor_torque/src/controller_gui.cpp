/*
 * controller_gui.cpp
 *
 * Motor Torque Controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <atc_motor_torque/controller_gui.h>

bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    gui->get_widget("torque_A_hscale", torque_A_hscale);
    gui->get_widget("torque_B_hscale", torque_B_hscale);
    gui->get_widget("torque_hip_hscale", torque_hip_hscale);

    if (torque_A_hscale && torque_B_hscale && torque_hip_hscale) {
        //torque_A_hscale->set_range(MIN_MTR_TRQ_CMD, MAX_MTR_TRQ_CMD);
        //torque_B_hscale->set_range(MIN_MTR_TRQ_CMD, MAX_MTR_TRQ_CMD);
        //torque_hip_hscale->set_range(MIN_HIP_MTR_TRQ_CMD, MAX_HIP_MTR_TRQ_CMD);
        torque_A_hscale->set_range(-10., 10.);
        torque_B_hscale->set_range(-10., 10.);
        torque_hip_hscale->set_range(-10., 10.);

        // Set up subscriber and publisher.
        sub = nh.subscribe("atc_motor_torque_status", 0, controllerCallback);
        pub = nh.advertise<atc_motor_torque::controller_input>("atc_motor_torque_input", 0);
        return true;
    }
    return false;
}

void controllerCallback(const atc_motor_torque::controller_status &status) {
    controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
    // Get parameters in the atrias_gui namespace.
    nh.getParam("/atrias_gui/torque_A", torque_A_param);
    nh.getParam("/atrias_gui/torque_B", torque_B_param);
    nh.getParam("/atrias_gui/torque_hip", torque_hip_param);

    // Configure the GUI.
    torque_A_hscale->set_value(torque_A_param);
    torque_B_hscale->set_value(torque_B_param);
    torque_hip_hscale->set_value(torque_hip_param);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    nh.setParam("/atrias_gui/torque_A", torque_A_param);
    nh.setParam("/atrias_gui/torque_B", torque_B_param);
    nh.setParam("/atrias_gui/torque_hip", torque_hip_param);
}

//! \brief Update the GUI.
void guiUpdate() {
    controllerDataOut.des_motor_torque_A   = torque_A_param   = torque_A_hscale->get_value();
    controllerDataOut.des_motor_torque_B   = torque_B_param   = torque_B_hscale->get_value();
    controllerDataOut.des_motor_torque_hip = torque_hip_param = torque_hip_hscale->get_value();
    pub.publish(controllerDataOut);
}

void guiTakedown() {
}
