/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_umich_1/controller_input.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Publisher pub;

// Data
atc_umich_1::controller_input controllerDataOut;

// GUI elements
Gtk::HScale *q1r_hscale,
        *q2r_hscale,
        *q3r_hscale,
        *q1l_hscale,
        *q2l_hscale,
        *q3l_hscale;

Gtk::SpinButton *kp1_hscale,
        *kp2_hscale,
        *kp3_hscale,
        *kd1_hscale,
        *kd2_hscale,
        *kd3_hscale,
        *epsilon_spinbutton;

#endif /* CONTROLLER_GUI_H_ */

