/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_sim_test/controller_input.h>
#include <atc_sim_test/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_sim_test::controller_input controllerDataOut;
atc_sim_test::controller_status controllerDataIn;

// GUI elements
Gtk::HScale *torque_A_hscale,
        *torque_B_hscale,
        *torque_hip_hscale;

Gtk::CheckButton *set_position_checkbutton;

// Parameters
//     NOTE: ROS parameters cannot be floats so loss-of-precision issues do not
//     arise when both C and python nodes access the same parameter. See:
//     http://answers.ros.org/question/10938/why-cant-you-use-floats-for-accessing-parameters-in-roscpp/
double torque_A_param;
double torque_B_param;
double torque_hip_param;

void controllerCallback(const atc_sim_test::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

