/*
 * controller_gui.h
 *
 *  Created on: April 1, 2014
 *      Author: Mikhail S. Jones
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_stabilized_standing/controller_input.h>
#include <atc_stabilized_standing/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_stabilized_standing::controller_input controllerDataOut;
atc_stabilized_standing::controller_status controllerDataIn;

// GUI elements
Gtk::ComboBox *main_controller_combobox;

Gtk::SpinButton *leg_pos_kp_spinbutton,
	*leg_pos_kd_spinbutton,
	*hip_pos_kp_spinbutton,
	*hip_pos_kd_spinbutton,
	*left_toe_pos_spinbutton,
	*right_toe_pos_spinbutton,
	*current_limit_spinbutton,
	*deflection_limit_spinbutton,
	*velocity_limit_spinbutton;

void controllerCallback(const atc_stabilized_standing::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

