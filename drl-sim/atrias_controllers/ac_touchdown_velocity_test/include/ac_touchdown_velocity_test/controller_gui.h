/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <ac_touchdown_velocity_test/controller_common.h>
#include <atrias_control/gui_library.h>
#include <ros/package.h>

Gtk::Label  *status_label;

Gtk::SpinButton *stance_p_gain,
                *stance_d_gain,
                *flight_p_gain,
                *flight_d_gain,
                *hip_p_gain,
                *hip_d_gain,
                *accel_p_gain,
                *accel_d_gain,
                *desired_leg_length,
                *acceleration_time,
                *touchdown_angle,
                *takeoff_height,
                *touchdown_height;

#endif /* CONTROLLER_GUI_H_ */
