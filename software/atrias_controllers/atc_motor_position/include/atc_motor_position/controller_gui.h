/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_motor_position/controller_input.h>
#include <atc_motor_position/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_motor_position::controller_input controllerDataOut;
atc_motor_position::controller_status controllerDataIn;

// GUI elements
Gtk::HScale *position_A_hscale,
        *position_B_hscale,
        *p_hscale,
        *d_hscale;

Gtk::CheckButton *set_position_checkbutton;

// Parameters
float a_position_param;
float b_position_param;
float p_gain_param;
float d_gain_param;

void controllerCallback(const atc_motor_position::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */
