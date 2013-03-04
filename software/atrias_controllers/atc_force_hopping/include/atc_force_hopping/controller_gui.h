/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include "atc_force_hopping/common.hpp"
#include <atc_force_hopping/controller_input.h>
#include <atc_force_hopping/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

using namespace atrias::controller;

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_force_hopping::controller_input controllerDataOut;
atc_force_hopping::controller_status controllerDataIn;

// GUI elements
Gtk::HScale       *flightLegLen;

Gtk::SpinButton   *flightP,
                  *flightD,
                  *stanceP,
                  *stanceD,
                  *hipP,
                  *hipD;

Gtk::ComboBox     *stancePSrc,
                  *stanceDSrc;

Gtk::Label        *stateLbl;

Gtk::ToggleButton *lockLeg;

void controllerCallback(const atc_force_hopping::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

// vim: noexpandtab
