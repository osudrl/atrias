/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_force_hopping/controller_input.h>
#include <atc_force_hopping/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_force_hopping::controller_input controllerDataOut;
atc_force_hopping::controller_status controllerDataIn;

// GUI elements
Gtk::HScale     *flightLegLen,
                *retractDiff;

Gtk::SpinButton *flightP,
                *flightD,
                *stanceP,
                *stanceD;

Gtk::ComboBox   *stancePSrc,
                *stanceDSrc,
                *legMode;

Gtk::Button     *shutdownBtn;

void controllerCallback(const atc_force_hopping::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

// vim: noexpandtab
