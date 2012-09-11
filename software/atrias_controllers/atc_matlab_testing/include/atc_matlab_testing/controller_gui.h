/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_matlab_testing/controller_input.h>
#include <atc_matlab_testing/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_matlab_testing::controller_input controllerDataOut;
atc_matlab_testing::controller_status controllerDataIn;

// GUI elements
Gtk::HScale *leg_length_hscale,
        *leg_angle_hscale,
        *p_leg_position_hscale,
        *d_leg_position_hscale,
        *hip_position_ang,
        *hip_position_p,
        *hip_position_d;

Gtk::CheckButton *update_checkbutton;

void controllerCallback(const atc_matlab_testing::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

