/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Ryan Van Why
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_velocity_tuning/controller_input.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Publisher pub;

// Data
atc_velocity_tuning::controller_input controllerDataOut;

// GUI elements
Gtk::SpinButton *minPosSpnBtn,
                *maxPosSpnBtn,
                *desVelSpnBtn,
                *kpSpnBtn;

Gtk::ToggleButton *sensorToggle,
                  *haltToggle;

Gtk::RadioButton  *absEncoder,
                  *incEncoder,
			   *forward,
			   *reverse;

#endif /* CONTROLLER_GUI_H_ */

