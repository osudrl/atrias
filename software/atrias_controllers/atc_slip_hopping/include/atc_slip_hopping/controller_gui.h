/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_slip_hopping/controller_input.h>
#include <atc_slip_hopping/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_slip_hopping::controller_input controllerDataOut;
atc_slip_hopping::controller_status controllerDataIn;

// GUI elements
Gtk::SpinButton *slip_spring_spinbutton,
	*standing_leg_spinbutton,
	*hop_height_spinbutton,
	*slip_leg_spinbutton,
	*leg_pos_kp_spinbutton,
	*leg_for_kp_spinbutton,
	*leg_pos_kd_spinbutton,
	*leg_for_kd_spinbutton,
	*hip_pos_kp_spinbutton,
	*hip_pos_kd_spinbutton;
	
Gtk::ComboBox *main_controller_combobox,
	*spring_type_combobox,
	*force_type_combobox,
	*stance_controller_combobox,
	*hop_type_combobox;

void controllerCallback(const atc_slip_hopping::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

