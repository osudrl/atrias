/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <ac_test/controller_common.h>
#include <atrias_control/gui_library.h>
#include <ros/package.h>

Gtk::Label  *status_label;

Gtk::SpinButton *stance_p_gain,
                *stance_d_gain,
                *flight_p_gain,
                *flight_d_gain,
                *hip_p_gain,
                *hip_d_gain,
                *desired_leg_length,
                *leg_angle,
                *takeoff_height,
                *touchdown_height;

#endif /* CONTROLLER_GUI_H_ */
