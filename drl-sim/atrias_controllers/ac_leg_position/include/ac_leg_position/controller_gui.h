/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <ac_leg_position/controller_common.h>
#include <drl_library/drl_math.h>
#include <atrias_control/gui_library.h>

Gtk::HScale *leg_length_hscale,
        *leg_angle_hscale,
        *p_leg_position_hscale,
        *d_leg_position_hscale,
        *hip_position_ang,
        *hip_position_p,
        *hip_position_d;

Gtk::CheckButton *update_checkbutton;

#endif /* CONTROLLER_GUI_H_ */
