/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_velocity_tuning/controller_input.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

ros::NodeHandle nh;
ros::Publisher pub;

atc_velocity_tuning::controller_input controllerDataOut;

Gtk::HScale *torque_A_hscale,
        *torque_B_hscale,
        *torque_hip_hscale;

Gtk::CheckButton *set_position_checkbutton;


#endif /* CONTROLLER_GUI_H_ */
