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
    gui->get_widget("dc_oscillate_frequency_spinbutton", dc_oscillate_frequency_spinbutton);
    gui->get_widget("dc_oscillate_enable_checkbutton", dc_oscillate_enable_checkbutton);
    gui->get_widget("dc_controller_combobox",  dc_controller_combobox);
    gui->get_widget("ip_spinbutton",           ip_spinbutton);
    gui->get_widget("ic_spinbutton",           ic_spinbutton);
    gui->get_widget("tp_spinbutton",           tp_spinbutton);
    gui->get_widget("tc_spinbutton",           tc_spinbutton);
    gui->get_widget("dc_signal_frequency_spinbutton", dc_signal_frequency_spinbutton);
    gui->get_widget("dc_signal_duty_cycle_spinbutton", dc_signal_duty_cycle_spinbutton);
    gui->get_widget("dc_spring_stiffness_spinbutton", dc_spring_stiffness_spinbutton);
    gui->get_widget("dc_hop_height_spinbutton", dc_hop_height_spinbutton);
    gui->get_widget("dc_test_togglebutton",    dc_test_togglebutton);
    gui->get_widget("cur_lim_togglebutton",    cur_lim_togglebutton);

    if (torque_left_A_hscale && torque_left_B_hscale && torque_left_hip_hscale &&
        torque_right_A_hscale && torque_right_B_hscale && torque_right_hip_hscale &&
        dc_oscillate_frequency_spinbutton &&
        dc_oscillate_enable_checkbutton &&
        dc_controller_combobox &&
        ip_spinbutton && ic_spinbutton && tp_spinbutton && tc_spinbutton &&
        dc_signal_frequency_spinbutton &&
        dc_signal_duty_cycle_spinbutton &&
        dc_spring_stiffness_spinbutton &&
        dc_hop_height_spinbutton &&
        dc_test_togglebutton && cur_lim_togglebutton) {
        torque_left_A_hscale->set_range(-30, 30);
        torque_left_B_hscale->set_range(-30, 30);
        torque_left_hip_hscale->set_range(-30, 30);
        torque_right_A_hscale->set_range(-120, 120);
        torque_right_B_hscale->set_range(-30, 30);
        torque_right_hip_hscale->set_range(-30, 30);
        dc_oscillate_frequency_spinbutton->set_range(0.0, 10.0);
        ip_spinbutton->set_range(0.0, 120.0);
        ic_spinbutton->set_range(0.0, 120.0);
        tp_spinbutton->set_range(0.0, 10.0);
        tc_spinbutton->set_range(0.0, 20.0);
        dc_signal_frequency_spinbutton->set_range(0.0, 10.0);
        dc_signal_duty_cycle_spinbutton->set_range(0.0, 1.0);
        dc_spring_stiffness_spinbutton->set_range(0.0, 100.0);
        dc_hop_height_spinbutton->set_range(0.0, 0.5);

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

    // Duty cycle tester values
    controllerDataOut.dc_mode = dc_controller_combobox->get_active_row_number();
    controllerDataOut.dc_freq = dc_signal_frequency_spinbutton->get_value();
    controllerDataOut.dc_dc   = dc_signal_duty_cycle_spinbutton->get_value();
    controllerDataOut.dc_spring_stiffness = dc_spring_stiffness_spinbutton->get_value();
    controllerDataOut.dc_hop_height = dc_hop_height_spinbutton->get_value();
    controllerDataOut.dc_oscillate = (dc_oscillate_enable_checkbutton->get_active()) ? true : false;
    controllerDataOut.dc_oscillate_freq = dc_oscillate_frequency_spinbutton->get_value();

    controllerDataOut.dc_ip = ip_spinbutton->get_value();
    controllerDataOut.dc_ic = ic_spinbutton->get_value();
    controllerDataOut.dc_tp = tp_spinbutton->get_value();
    controllerDataOut.dc_tc = tc_spinbutton->get_value();

    controllerDataOut.dutyCycleTest = dc_test_togglebutton->get_active();
    controllerDataOut.limitCurrent = cur_lim_togglebutton->get_active();
    pub.publish(controllerDataOut);
}

void guiTakedown() {
}
