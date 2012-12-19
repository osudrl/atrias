/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Ryan Van Why
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_hip_force_test/controller_input.h>
#include <atc_hip_force_test/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_hip_force_test::controller_input controllerDataOut;
atc_hip_force_test::controller_status controllerDataIn;

// GUI elements
Gtk::SpinButton *flightP,
                *flightD,
                *stanceP,
                *stanceD,
                *toeFilterGain,
                *toeThreshold,
                *legLen,
                *force;

Gtk::Label *stateLbl;

void controllerCallback(const atc_hip_force_test::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

// vim: noexpandtab