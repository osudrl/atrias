/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <ac_motor_position/controller_common.h>
#include <atrias_control/gui_library.h>

Gtk::HScale *position_A_hscale,
        *position_B_hscale,
        *p_hscale,
        *d_hscale;

Gtk::CheckButton *set_position_checkbutton;

#endif /* CONTROLLER_GUI_H_ */
