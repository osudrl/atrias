/*
 * controller_gui.h
 *
 *  Created on: Jan 31, 2013
 *      Author: Mikhail Jones
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_single_leg_hopping/controller_input.h>
#include <atc_single_leg_hopping/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_single_leg_hopping::controller_input controllerDataOut;
atc_single_leg_hopping::controller_status controllerDataIn;

// GUI elements
Gtk::SpinButton *flight_leg_P_gain_spinbutton,
                *flight_leg_D_gain_spinbutton,
                *flight_hip_P_gain_spinbutton,
                *flight_hip_D_gain_spinbutton,
                *stance_leg_P_gain_spinbutton,
                *stance_leg_D_gain_spinbutton,
                *stance_hip_P_gain_spinbutton,
                *stance_hip_D_gain_spinbutton,
                *fy_spinbutton,
                *fz_spinbutton,
                *slip_height_spinbutton,
                *slip_mass_spinbutton,
                *slip_spring_spinbutton,
                *slip_leg_spinbutton,
                *hip_angle_spinbutton,
                *left_hip_target_spinbutton,
                *right_hip_target_spinbutton,
                *robot_spring_spinbutton,
                *robot_motor_spinbutton;

Gtk::ToggleButton *constant_force_togglebutton,
                *slip_force_togglebutton,
                *constant_hip_togglebutton,
                *advanced_hip_togglebutton,
                *debug1_togglebutton,
                *debug2_togglebutton,
                *debug3_togglebutton,
                *debug4_togglebutton;

// Parameters
//     NOTE: ROS parameters cannot be floats so loss-of-precision issues do not
//     arise when both C and python nodes access the same parameter. See:
//     http://answers.ros.org/question/10938/why-cant-you-use-floats-for-accessing-parameters-in-roscpp/

void controllerCallback(const atc_single_leg_hopping::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

