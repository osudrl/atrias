/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_canonical_walking/controller_input.h>
#include <atc_canonical_walking/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// This controller's common definitions
#include "common.hpp"

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_canonical_walking::controller_input controllerDataOut;
atc_canonical_walking::controller_status controllerDataIn;

// GUI elements
Gtk::ComboBox *main_controller_combobox;
    
Gtk::SpinButton *leg_pos_kp_spinbutton,
    *leg_pos_kd_spinbutton,
    *hip_pos_kp_spinbutton,
    *left_toe_pos_spinbutton,
    *hip_pos_kd_spinbutton,
    *right_toe_pos_spinbutton;

Gtk::CheckButton *tau_control_checkbutton;
Gtk::HScale      *tau_hscale;

void controllerCallback(const atc_canonical_walking::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

