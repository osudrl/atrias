/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_slip_walking/controller_input.h>
#include <atc_slip_walking/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_slip_walking::controller_input controllerDataOut;
atc_slip_walking::controller_status controllerDataIn;

// Math
double hipIn, hipOut, hipCenter;

// GUI elements
Gtk::HScale *position_left_A_hscale,
            *position_left_B_hscale,
            *position_left_hip_hscale,
            *position_right_A_hscale,
            *position_right_B_hscale,
            *position_right_hip_hscale;

Gtk::SpinButton *position_leg_motor_p_spinbutton,
                *position_leg_motor_d_spinbutton,
                *position_hip_motor_p_spinbutton,
                *position_hip_motor_d_spinbutton;

Gtk::CheckButton *set_leg_motor_position_checkbutton;
Gtk::CheckButton *set_hip_motor_position_checkbutton;

void controllerCallback(const atc_slip_walking::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */
