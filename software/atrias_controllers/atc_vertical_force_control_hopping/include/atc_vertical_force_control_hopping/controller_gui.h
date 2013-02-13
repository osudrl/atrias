/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_vertical_force_control_hopping/controller_input.h>
#include <atc_vertical_force_control_hopping/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_vertical_force_control_hopping::controller_input controllerDataOut;
atc_vertical_force_control_hopping::controller_status controllerDataIn;

// GUI elements


// Parameters
//     NOTE: ROS parameters cannot be floats so loss-of-precision issues do not
//     arise when both C and python nodes access the same parameter. See:
//     http://answers.ros.org/question/10938/why-cant-you-use-floats-for-accessing-parameters-in-roscpp/

void controllerCallback(const atc_vertical_force_control_hopping::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

