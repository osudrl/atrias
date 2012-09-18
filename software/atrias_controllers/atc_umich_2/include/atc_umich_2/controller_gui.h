/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_umich_2/controller_input.h>
#include <atc_umich_2/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_umich_2::controller_input controllerDataOut;
atc_umich_2::controller_status controllerDataIn;

// GUI elements
Gtk::ComboBox *s_mode_combo_box;

Gtk::SpinButton *s_freq_spinbutton,
        *torso_offset_spinbutton,
        *kp1_spinbutton,
        *kp2_spinbutton,
        *kp3_spinbutton,
        *kd1_spinbutton,
        *kd2_spinbutton,
        *kd3_spinbutton,
        *leg_saturation_cap_spinbutton,
        *hip_saturation_cap_spinbutton,
        *epsilon_spinbutton,

	*y1l_spinbutton,
	*y2l_spinbutton,
	*y3l_spinbutton,
	*y1r_spinbutton,
	*y2r_spinbutton,
	*y3r_spinbutton,
	*dy1l_spinbutton,
	*dy2l_spinbutton,
	*dy3l_spinbutton,
	*dy1r_spinbutton,
	*dy2r_spinbutton,
	*dy3r_spinbutton;
	*s_spinbutton;
	*ds_spinbutton;

void controllerCallback(const atc_umich_2::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

