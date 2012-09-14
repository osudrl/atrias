/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_motor_torque_sin/controller_input.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

ros::NodeHandle nh;

ros::Publisher pub;

atc_motor_torque_sin::controller_input controllerDataOut;

Gtk::ComboBox *motor_selection_combobox;

Gtk::SpinButton *motor_offset_spinbutton;

Gtk::HScale *angle_amplitude_hscale,
            *angle_frequency_hscale,
            *length_amplitude_hscale,
            *length_frequency_hscale;

#endif /* CONTROLLER_GUI_H_ */
