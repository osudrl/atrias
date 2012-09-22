/*
 * controller_gui.h
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_demo_range_of_motion/controller_input.h>
#include <atc_demo_range_of_motion/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_demo_range_of_motion::controller_input controllerDataOut;
atc_demo_range_of_motion::controller_status controllerDataIn;

// GUI elements
Gtk::SpinButton *position_left_A_spinbutton,
                *position_left_B_spinbutton,
                *position_left_hip_spinbutton,
                *position_right_A_spinbutton,
                *position_right_B_spinbutton,
                *position_right_hip_spinbutton,
                *position_leg_motor_p_spinbutton,
                *position_leg_motor_d_spinbutton,
                *position_hip_motor_p_spinbutton,
                *position_hip_motor_d_spinbutton,
                *position_leg_duration_spinbutton,
                *position_hip_duration_spinbutton;

Gtk::ComboBox *position_mode_combobox;

void controllerCallback(const atc_demo_range_of_motion::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

