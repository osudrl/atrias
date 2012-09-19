/*
 * controller_gui.cpp
 *
 * atc_umich_2 controller
 *
 *  Created on: September 17, 2012
 *      Author: Soo-Hyun Yoo
 */

#include <atc_umich_2/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    gui->get_widget("kp1_spinbutton", kp1_spinbutton);
    gui->get_widget("kp2_spinbutton", kp2_spinbutton);
    gui->get_widget("kp3_spinbutton", kp3_spinbutton);
    gui->get_widget("kd1_spinbutton", kd1_spinbutton);
    gui->get_widget("kd2_spinbutton", kd2_spinbutton);
    gui->get_widget("kd3_spinbutton", kd3_spinbutton);
    gui->get_widget("leg_saturation_cap_spinbutton", leg_saturation_cap_spinbutton);
    gui->get_widget("hip_saturation_cap_spinbutton", hip_saturation_cap_spinbutton);
    gui->get_widget("epsilon_spinbutton", epsilon_spinbutton);

    gui->get_widget("s_freq_spinbutton", s_freq_spinbutton);
    gui->get_widget("s_mode_combo_box", s_mode_combo_box);
    gui->get_widget("torso_offset_spinbutton", torso_offset_spinbutton);
    gui->get_widget("swap_type_combo_box", swap_type_combo_box);
    gui->get_widget("s_threshold_spinbutton", s_threshold_spinbutton);
    gui->get_widget("switch_spring_threshold_spinbutton", switch_spring_threshold_spinbutton);
    gui->get_widget("stance_spring_threshold_spinbutton", stance_spring_threshold_spinbutton);
    gui->get_widget("scuff_1_spinbutton", scuff_1_spinbutton);
    gui->get_widget("scuff_2_spinbutton", scuff_2_spinbutton);

    gui->get_widget("y1l_spinbutton", y1l_spinbutton);
    gui->get_widget("y2l_spinbutton", y2l_spinbutton);
    gui->get_widget("y3l_spinbutton", y3l_spinbutton);
    gui->get_widget("y1r_spinbutton", y1r_spinbutton);
    gui->get_widget("y2r_spinbutton", y2r_spinbutton);
    gui->get_widget("y3r_spinbutton", y3r_spinbutton);
    gui->get_widget("dy1l_spinbutton", dy1l_spinbutton);
    gui->get_widget("dy2l_spinbutton", dy2l_spinbutton);
    gui->get_widget("dy3l_spinbutton", dy3l_spinbutton);
    gui->get_widget("dy1r_spinbutton", dy1r_spinbutton);
    gui->get_widget("dy2r_spinbutton", dy2r_spinbutton);
    gui->get_widget("dy3r_spinbutton", dy3r_spinbutton);
   
    gui->get_widget("s_spinbutton", s_spinbutton);
    gui->get_widget("ds_spinbutton", ds_spinbutton);
    gui->get_widget("q3l_des_spinbutton", q3l_des_spinbutton);
    gui->get_widget("q3r_des_spinbutton", q3r_des_spinbutton);

    gui->get_widget("left_support_leg_radiobutton", left_support_leg_radiobutton);
    gui->get_widget("right_support_leg_radiobutton", right_support_leg_radiobutton);

    if (kp1_spinbutton && kp2_spinbutton && kp3_spinbutton &&
        kd1_spinbutton && kd2_spinbutton && kd3_spinbutton &&
        leg_saturation_cap_spinbutton && hip_saturation_cap_spinbutton &&
        epsilon_spinbutton && torso_offset_spinbutton &&
        y1l_spinbutton && y2l_spinbutton && y3l_spinbutton &&
        y1r_spinbutton && y2r_spinbutton && y3r_spinbutton &&
        dy1l_spinbutton && dy2l_spinbutton && dy3l_spinbutton &&
        dy1r_spinbutton && dy2r_spinbutton && dy3r_spinbutton &&
        s_mode_combo_box && s_freq_spinbutton && s_spinbutton &&
	ds_spinbutton && left_support_leg_radiobutton && right_support_leg_radiobutton &&
        q3l_des_spinbutton && q3r_des_spinbutton && swap_type_combo_box &&
        s_threshold_spinbutton && switch_spring_threshold_spinbutton &&
        stance_spring_threshold_spinbutton && scuff_1_spinbutton &&
        scuff_2_spinbutton) {
        // Set ranges.
        kp1_spinbutton->set_range(0, 500);
        kp2_spinbutton->set_range(0, 500);
        kp3_spinbutton->set_range(0, 500);
        kd1_spinbutton->set_range(0, 100);
        kd2_spinbutton->set_range(0, 100);
        kd3_spinbutton->set_range(0, 100);
        leg_saturation_cap_spinbutton->set_range(0, 60);
        hip_saturation_cap_spinbutton->set_range(0, 10);
        epsilon_spinbutton->set_range(0, 1);

        torso_offset_spinbutton->set_range(-20., 20.);
        s_freq_spinbutton->set_range(0., 2.);
        right_support_leg_radiobutton->set_active(true);
        s_threshold_spinbutton->set_range(0.5, 1.5);
        switch_spring_threshold_spinbutton->set_range(-5., 5.);
        stance_spring_threshold_spinbutton->set_range(-5., 5.);
        scuff_1_spinbutton->set_range(0., 1.);
        scuff_2_spinbutton->set_range(0., 2.);

        y1r_spinbutton->set_range(-10000, 10000);
        y2r_spinbutton->set_range(-10000, 10000);
        y3r_spinbutton->set_range(-10000, 10000);
        y1l_spinbutton->set_range(-10000, 10000);
        y2l_spinbutton->set_range(-10000, 10000);
        y3l_spinbutton->set_range(-10000, 10000);
        dy1r_spinbutton->set_range(-10000, 10000);
        dy2r_spinbutton->set_range(-10000, 10000);
        dy3r_spinbutton->set_range(-10000, 10000);
        dy1l_spinbutton->set_range(-10000, 10000);
        dy2l_spinbutton->set_range(-10000, 10000);
        dy3l_spinbutton->set_range(-10000, 10000);
        s_spinbutton->set_range(-10000, 10000);
        ds_spinbutton->set_range(-10000, 10000);
        q3l_des_spinbutton->set_range(-5, 5);
        q3r_des_spinbutton->set_range(-5, 5);

        // Set increments.
        kp1_spinbutton->set_increments(0.1, 1.0);
        kp2_spinbutton->set_increments(0.1, 1.0);
        kp3_spinbutton->set_increments(0.1, 1.0);
        kd1_spinbutton->set_increments(0.1, 1.0);
        kd2_spinbutton->set_increments(0.1, 1.0);
        kd3_spinbutton->set_increments(0.1, 1.0);
        leg_saturation_cap_spinbutton->set_increments(0.1, 1.0);
        hip_saturation_cap_spinbutton->set_increments(0.1, 1.0);
        epsilon_spinbutton->set_increments(0.01, 0.1);

        // Set default values
        s_threshold_spinbutton->set_value(0.9);
        stance_spring_threshold->set_value(-1.0);
        swing_spring_threshold->set_value(-0.2);

        // Set up subscriber and publisher.
        sub = nh.subscribe("atc_umich_2_status", 0, controllerCallback);
        pub = nh.advertise<atc_umich_2::controller_input>("atc_umich_2_input", 0);
        return true;
    }
    return false;
}

void controllerCallback(const atc_umich_2::controller_status &status) {
    controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
    // Get parameters in the atrias_gui namespace.
	int s_mode_tmp;
    nh.getParam("/atrias_gui/s_mode", s_mode_tmp);
    controllerDataOut.s_mode = (uint8_t)s_mode_tmp;
    nh.getParam("/atrias_gui/s_freq", controllerDataOut.s_freq);
    nh.getParam("/atrias_gui/kp1", controllerDataOut.kp1);
    nh.getParam("/atrias_gui/kp2", controllerDataOut.kp2);
    nh.getParam("/atrias_gui/kp3", controllerDataOut.kp3);
    nh.getParam("/atrias_gui/kd1", controllerDataOut.kd1);
    nh.getParam("/atrias_gui/kd2", controllerDataOut.kd2);
    nh.getParam("/atrias_gui/kd3", controllerDataOut.kd3);
    nh.getParam("/atrias_gui/leg_saturation_cap", controllerDataOut.leg_saturation_cap);
    nh.getParam("/atrias_gui/hip_saturation_cap", controllerDataOut.hip_saturation_cap);
    nh.getParam("/atrias_gui/epsilon", controllerDataOut.epsilon);
    nh.getParam("/atrias_gui/torso_offset", controllerDataOut.torso_offset);

    // Configure the GUI.
    s_mode_combo_box->set_active(controllerDataOut.s_mode);
    s_freq_spinbutton->set_value(controllerDataOut.s_freq);
    kp1_spinbutton->set_value(controllerDataOut.kp1);
    kp2_spinbutton->set_value(controllerDataOut.kp2);
    kp3_spinbutton->set_value(controllerDataOut.kp3);
    kd1_spinbutton->set_value(controllerDataOut.kd1);
    kd2_spinbutton->set_value(controllerDataOut.kd2);
    kd3_spinbutton->set_value(controllerDataOut.kd3);
    leg_saturation_cap_spinbutton->set_value(controllerDataOut.leg_saturation_cap);
    hip_saturation_cap_spinbutton->set_value(controllerDataOut.hip_saturation_cap);
    epsilon_spinbutton->set_value(controllerDataOut.epsilon);
    torso_offset_spinbutton->set_value(controllerDataOut.torso_offset);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    nh.setParam("/atrias_gui/s_mode", controllerDataOut.s_mode);
    nh.setParam("/atrias_gui/s_freq", controllerDataOut.s_freq);
    nh.setParam("/atrias_gui/kp1", controllerDataOut.kp1);
    nh.setParam("/atrias_gui/kp2", controllerDataOut.kp2);
    nh.setParam("/atrias_gui/kp3", controllerDataOut.kp3);
    nh.setParam("/atrias_gui/kd1", controllerDataOut.kd1);
    nh.setParam("/atrias_gui/kd2", controllerDataOut.kd2);
    nh.setParam("/atrias_gui/kd3", controllerDataOut.kd3);
    nh.setParam("/atrias_gui/leg_saturation_cap", controllerDataOut.leg_saturation_cap);
    nh.setParam("/atrias_gui/hip_saturation_cap", controllerDataOut.hip_saturation_cap);
    nh.setParam("/atrias_gui/epsilon", controllerDataOut.epsilon);
    nh.setParam("/atrias_gui/torso_offset", controllerDataOut.torso_offset);
}

//! \brief Update the GUI.
void guiUpdate() {
    // Update controller status values
    y1l_spinbutton->set_value(controllerDataIn.yl[0]);
    y2l_spinbutton->set_value(controllerDataIn.yl[1]);
    y3l_spinbutton->set_value(controllerDataIn.yl[2]);
    y1r_spinbutton->set_value(controllerDataIn.yr[0]);
    y2r_spinbutton->set_value(controllerDataIn.yr[1]);
    y3r_spinbutton->set_value(controllerDataIn.yr[2]);
    dy1l_spinbutton->set_value(controllerDataIn.dyl[0]);
    dy2l_spinbutton->set_value(controllerDataIn.dyl[1]);
    dy3l_spinbutton->set_value(controllerDataIn.dyl[2]);
    dy1r_spinbutton->set_value(controllerDataIn.dyr[0]);
    dy2r_spinbutton->set_value(controllerDataIn.dyr[1]);
    dy3r_spinbutton->set_value(controllerDataIn.dyr[2]);
    s_spinbutton->set_value(controllerDataIn.s);
    ds_spinbutton->set_value(controllerDataIn.ds);

    controllerDataOut.s_mode = (uint8_t)s_mode_combo_box->get_active_row_number();
    controllerDataOut.s_freq = s_freq_spinbutton->get_value();
    controllerDataOut.kp1 = kp1_spinbutton->get_value();
    controllerDataOut.kp2 = kp2_spinbutton->get_value();
    controllerDataOut.kp3 = kp3_spinbutton->get_value();
    controllerDataOut.kd1 = kd1_spinbutton->get_value();
    controllerDataOut.kd2 = kd2_spinbutton->get_value();
    controllerDataOut.kd3 = kd3_spinbutton->get_value();
    controllerDataOut.leg_saturation_cap = leg_saturation_cap_spinbutton->get_value();
    controllerDataOut.hip_saturation_cap = hip_saturation_cap_spinbutton->get_value();
    controllerDataOut.epsilon = epsilon_spinbutton->get_value();
    controllerDataOut.torso_offset = torso_offset_spinbutton->get_value();
    controllerDataOut.stance_leg = left_support_leg_radiobutton->get_active();
    controllerDataOut.q3_des[0] = q3r_des_spinbutton->get_value();
    controllerDataOut.q3_des[1] = q3l_des_spinbutton->get_value();
    controllerDataOut.swap_threshold[0] = s_threshold_spinbutton->get_value();
    controllerDataOut.swap_threshold[1] = stance_spring_threshold_spinbutton->get_value();
    controllerDataOut.swap_threshold[2] = switch_spring_threshold_spinbutton->get_value();
    controllerDataOut.scuff[0] = scuff_1_spinbutton->get_value();
    controllerDataOut.scuff[1] = scuff_2_spinbutton->get_value();
    controllerDataOut.swap = (uint8_t)swap_type_combo_box->get_active_row_number();
    pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

