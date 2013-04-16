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
    gui->get_widget("torque_left_A_hscale",    torque_left_A_hscale);
    gui->get_widget("torque_left_B_hscale",    torque_left_B_hscale);
    gui->get_widget("torque_left_hip_hscale",  torque_left_hip_hscale);
    gui->get_widget("torque_right_A_hscale",   torque_right_A_hscale);
    gui->get_widget("torque_right_B_hscale",   torque_right_B_hscale);
    gui->get_widget("torque_right_hip_hscale", torque_right_hip_hscale);

    if (torque_left_A_hscale && torque_left_B_hscale && torque_left_hip_hscale &&
        torque_right_A_hscale && torque_right_B_hscale && torque_right_hip_hscale) {
        //torque_left_A_hscale->set_range(MIN_MTR_CURRENT_CMD, MAX_MTR_CURRENT_CMD);
        //torque_left_B_hscale->set_range(MIN_MTR_CURRENT_CMD, MAX_MTR_CURRENT_CMD);
        //torque_left_hip_hscale->set_range(MIN_HIP_MTR_CURRENT_CMD, MAX_HIP_MTR_CURRENT_CMD);
        torque_left_A_hscale->set_range(-10., 10.);
        torque_left_B_hscale->set_range(-10., 10.);
        torque_left_hip_hscale->set_range(-10., 10.);
        torque_right_A_hscale->set_range(-10., 10.);
        torque_right_B_hscale->set_range(-10., 10.);
        torque_right_hip_hscale->set_range(-10., 10.);

        // Set up subscriber and publisher.
        pub = nh.advertise<atc_motor_torque::controller_input>("ATCMotorTorque_input", 0);
        return true;
    }
    return false;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
    // Get parameters in the atrias_gui namespace.
    nh.getParam("/atrias_gui/torque_left_A",    torque_left_A_param);
    nh.getParam("/atrias_gui/torque_left_B",    torque_left_B_param);
    nh.getParam("/atrias_gui/torque_left_hip",  torque_left_hip_param);
    nh.getParam("/atrias_gui/torque_right_A",   torque_right_A_param);
    nh.getParam("/atrias_gui/torque_right_B",   torque_right_B_param);
    nh.getParam("/atrias_gui/torque_right_hip", torque_right_hip_param);

    // Configure the GUI.
    torque_left_A_hscale->set_value(torque_left_A_param);
    torque_left_B_hscale->set_value(torque_left_B_param);
    torque_left_hip_hscale->set_value(torque_left_hip_param);
    torque_right_A_hscale->set_value(torque_right_A_param);
    torque_right_B_hscale->set_value(torque_right_B_param);
    torque_right_hip_hscale->set_value(torque_right_hip_param);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    nh.setParam("/atrias_gui/torque_left_A",    torque_left_A_param);
    nh.setParam("/atrias_gui/torque_left_B",    torque_left_B_param);
    nh.setParam("/atrias_gui/torque_left_hip",  torque_left_hip_param);
    nh.setParam("/atrias_gui/torque_right_A",   torque_right_A_param);
    nh.setParam("/atrias_gui/torque_right_B",   torque_right_B_param);
    nh.setParam("/atrias_gui/torque_right_hip", torque_right_hip_param);
}

//! \brief Update the GUI.
void guiUpdate() {
    controllerDataOut.des_motor_torque_left_A   = torque_left_A_param   = torque_left_A_hscale->get_value();
    controllerDataOut.des_motor_torque_left_B   = torque_left_B_param   = torque_left_B_hscale->get_value();
    controllerDataOut.des_motor_torque_left_hip = torque_left_hip_param = torque_left_hip_hscale->get_value();
    controllerDataOut.des_motor_torque_right_A   = torque_right_A_param   = torque_right_A_hscale->get_value();
    controllerDataOut.des_motor_torque_right_B   = torque_right_B_param   = torque_right_B_hscale->get_value();
    controllerDataOut.des_motor_torque_right_hip = torque_right_hip_param = torque_right_hip_hscale->get_value();
    pub.publish(controllerDataOut);
}

void guiTakedown() {
}
