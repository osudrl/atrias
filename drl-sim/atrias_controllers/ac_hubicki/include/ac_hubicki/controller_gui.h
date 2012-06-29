/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <ac_hubicki/controller_common.h>
#include <atrias_control/gui_library.h>

Gtk::Label *hubicki_state_label;

Gtk::HScale *hubicki_stance_hip_p_gain,
            *hubicki_stance_hip_d_gain,
            *hubicki_flight_hip_p_gain,
            *hubicki_flight_hip_d_gain;

Gtk::SpinButton *hubicki_desired_velocity_spinbutton,
                *hubicki_desired_height_spinbutton,
                *hubicki_hor_vel_gain_spinbutton,
                *hubicki_leg_force_gain_spinbutton,
                *hubicki_leg_angle_gain_spinbutton,
                *hubicki_stance_p_gain_spinbutton,
                *hubicki_stance_d_gain_spinbutton,
                *hubicki_stance_spring_threshold_spinbutton,
                *hubicki_preferred_leg_len_spinbutton,
                *hubicki_flight_p_gain_spinbutton,
                *hubicki_flight_d_gain_spinbutton,
                *hubicki_flight_spring_threshold_spinbutton;

#endif /* CONTROLLER_GUI_H_ */
