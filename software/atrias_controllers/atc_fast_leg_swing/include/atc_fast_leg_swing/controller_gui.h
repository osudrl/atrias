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
Gtk::SpinButton *frequency_spinbutton,
                *leg_magnitude_spinbutton,
                *hip_magnitude_spinbutton;

Gtk::RadioButton *sweep_radiobutton,
                 *extend_radiobutton;

void controllerCallback(const atc_fast_leg_swing::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

