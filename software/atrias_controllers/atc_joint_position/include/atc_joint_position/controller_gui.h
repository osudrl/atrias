/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_joint_position/controller_input.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Publisher pub;

// Data
atc_joint_position::controller_input controllerDataOut;

// GUI elements
Gtk::SpinButton *p_hl_spinbutton,
                *d_hl_spinbutton,
                *a_hl_spinbutton,
                *p_hr_spinbutton,
                *d_hr_spinbutton,
                *a_hr_spinbutton,
                *p_ll_spinbutton,
                *d_ll_spinbutton,
                *a_ll_spinbutton,
                *l_ll_spinbutton,
                *p_lr_spinbutton,
                *d_lr_spinbutton,
                *a_lr_spinbutton,
                *l_lr_spinbutton,
                *max_leg_spd_spinbutton,
                *max_hip_spd_spinbutton;

Gtk::CheckButton *vertical_checkbutton,
                 *sync_checkbutton,
                 *smooth_leg_motion_checkbutton,
                 *smooth_hip_motion_checkbutton;

// Parameters
//     NOTE: ROS parameters cannot be floats so loss-of-precision issues do not
//     arise when both C and python nodes access the same parameter. See:
//     http://answers.ros.org/question/10938/why-cant-you-use-floats-for-accessing-parameters-in-roscpp/


#endif /* CONTROLLER_GUI_H_ */

