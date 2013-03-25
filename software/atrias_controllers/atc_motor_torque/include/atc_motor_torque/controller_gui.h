/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_motor_torque/controller_input.h>
#include <atc_motor_torque/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_motor_torque::controller_input controllerDataOut;
atc_motor_torque::controller_status controllerDataIn;

// GUI elements
Gtk::HScale *torque_left_A_hscale,
        *torque_left_B_hscale,
        *torque_left_hip_hscale,
        *torque_right_A_hscale,
        *torque_right_B_hscale,
        *torque_right_hip_hscale;

Gtk::SpinButton *dc_oscillate_frequency_spinbutton,
        *dc_signal_frequency_spinbutton,
        *ip_spinbutton,
        *ic_spinbutton,
        *tp_spinbutton,
        *tc_spinbutton;

Gtk::ToggleButton *dc_test_togglebutton,
        *cur_lim_togglebutton;

Gtk::RadioButton *dc_square_wave_radiobutton,
        *dc_sine_wave_radiobutton;

Gtk::CheckButton *dc_oscillate_enable_checkbutton,
        *set_position_checkbutton;

// Parameters
//     NOTE: ROS parameters cannot be floats so loss-of-precision issues do not
//     arise when both C and python nodes access the same parameter. See:
//     http://answers.ros.org/question/10938/why-cant-you-use-floats-for-accessing-parameters-in-roscpp/
double torque_left_A_param;
double torque_left_B_param;
double torque_left_hip_param;
double torque_right_A_param;
double torque_right_B_param;
double torque_right_hip_param;

void controllerCallback(const atc_motor_torque::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

