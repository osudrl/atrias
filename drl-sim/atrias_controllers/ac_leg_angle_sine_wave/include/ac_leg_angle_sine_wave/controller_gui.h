/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <ac_leg_angle_sine_wave/controller_common.h>
#include <atrias_control/gui_library.h>

Gtk::HScale *leg_angle_amplitude_hscale,
            *leg_angle_frequency_hscale,
            *leg_length_amplitude_hscale,
            *leg_length_frequency_hscale,
            *p_sine_wave_hscale,
            *d_sine_wave_hscale;

#endif /* CONTROLLER_GUI_H_ */
