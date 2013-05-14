/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_heuristic_slip_walking/controller_input.h>
#include <atc_heuristic_slip_walking/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_heuristic_slip_walking::controller_input controllerDataOut;
atc_heuristic_slip_walking::controller_status controllerDataIn;

// GUI elements
Gtk::SpinButton *walking_state_spinbutton,
	*standing_leg_spinbutton,
	*slip_leg_spinbutton,
	*leg_pos_kp_spinbutton,
	*leg_for_kp_spinbutton,
	*leg_pos_kd_spinbutton,
	*leg_for_kd_spinbutton,
	*hip_pos_kp_spinbutton,
	*hip_pos_kd_spinbutton;
	
Gtk::ComboBox *main_controller_combobox,
	*state_trigger_combobox;

void controllerCallback(const atc_heuristic_slip_walking::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

