/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <ac_raibert/controller_common.h>
#include <atrias_control/gui_library.h>

Gtk::Label *raibert_state_label;

Gtk::HScale *raibert_stance_hip_p_gain,
            *raibert_stance_hip_d_gain,
            *raibert_flight_hip_p_gain,
            *raibert_flight_hip_d_gain;

Gtk::SpinButton *raibert_desired_velocity_spinbutton,
                *raibert_desired_height_spinbutton,
                *raibert_hor_vel_gain_spinbutton,
                *raibert_leg_force_gain_spinbutton,
                *raibert_leg_angle_gain_spinbutton,
                *raibert_stance_p_gain_spinbutton,
                *raibert_stance_d_gain_spinbutton,
                *raibert_stance_spring_threshold_spinbutton,
                *raibert_preferred_leg_len_spinbutton,
                *raibert_flight_p_gain_spinbutton,
                *raibert_flight_d_gain_spinbutton,
                *raibert_flight_spring_threshold_spinbutton;

#endif /* CONTROLLER_GUI_H_ */
