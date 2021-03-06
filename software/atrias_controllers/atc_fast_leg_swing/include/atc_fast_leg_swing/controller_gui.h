/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_fast_leg_swing/controller_input.h>
#include <atc_fast_leg_swing/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_fast_leg_swing::controller_input controllerDataOut;
atc_fast_leg_swing::controller_status controllerDataIn;

// GUI elements
Gtk::HScale *frequency_hscale,
            *leg_magnitude_hscale,
            *hip_magnitude_hscale;

Gtk::SpinButton *leg_p_spinbutton,
                *leg_d_spinbutton,
                *hip_p_spinbutton,
                *hip_d_spinbutton;

Gtk::RadioButton *sweep_radiobutton,
                 *extend_radiobutton;

Gtk::ToggleButton *demo_enable_togglebutton;

void controllerCallback(const atc_fast_leg_swing::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

