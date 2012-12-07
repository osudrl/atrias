/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Ryan Van Why
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_subcontroller_test/controller_input.h>
#include <atc_subcontroller_test/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

#include <string>
#include <boost/lexical_cast.hpp>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_subcontroller_test::controller_input controllerDataOut;
atc_subcontroller_test::controller_status controllerDataIn;

// GUI elements
Gtk::Entry *in1,
          *in2,
		*in3,
		*in4,
		*in5,
		*in6,
		*in7,
		*in8,
		*in9,
		*out1,
		*out2,
		*out3,
		*out4,
		*out5,
		*out6,
		*out7,
		*out8,
		*out9;

void controllerCallback(const atc_subcontroller_test::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

