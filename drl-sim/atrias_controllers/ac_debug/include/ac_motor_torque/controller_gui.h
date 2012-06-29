/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <ac_motor_torque/controller_common.h>
#include <atrias_control/gui_library.h>

Gtk::Label *command_label,
           *loop_time_label,
           *torque_A_label,
           *torque_B_label;

Gtk::Button *subtract_A_torque_button,
            *add_A_torque_button,
            *subtract_B_torque_button,
            *add_B_torque_button;

Gtk::ProgressBar *progress_bar;

Gtk::HScale *speed_hscale;

float torqueA;
float torqueB;

void add_a_torque();
void subtract_a_torque();
void add_b_torque();
void subtract_b_torque();

#endif /* GUI_PLUGIN_H_ */
