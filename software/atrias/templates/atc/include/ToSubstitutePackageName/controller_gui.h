/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <ToSubstitutePackageName/controller_input.h>
#include <ToSubstitutePackageName/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
ToSubstitutePackageName::controller_input controllerDataOut;
ToSubstitutePackageName::controller_status controllerDataIn;

// GUI elements

void controllerCallback(const ToSubstitutePackageName::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

