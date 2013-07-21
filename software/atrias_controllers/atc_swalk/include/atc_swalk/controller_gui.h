/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_swalk/controller_input.h>
#include <atc_swalk/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_swalk::controller_input controllerDataOut;
atc_swalk::controller_status controllerDataIn;

// GUI elements
Gtk::ComboBox *control_combobox;

Gtk::Button *gc_l_button,
	*gc_r_button;

Gtk::SpinButton *aea_spinbutton,
	*lhip_pos_spinbutton,
	*rhip_pos_spinbutton,
	*pea_spinbutton,
	*l_leg_fl_spinbutton,
	*lot_spinbutton,
	*d_af_spinbutton,
	*p_lf_spinbutton,
	*d_lf_spinbutton,
	*l_fl_spinbutton,
	*l_leg_st_spinbutton,
	*p_as_spinbutton,
	*d_as_spinbutton,
	*p_ls_spinbutton,
	*d_ls_spinbutton;

Gtk::HScale *l_st_hscale;

void controllerCallback(const atc_swalk::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

