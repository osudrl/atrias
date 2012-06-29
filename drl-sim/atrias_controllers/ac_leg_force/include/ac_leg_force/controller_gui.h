/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <ac_leg_force/controller_common.h>
#include <atrias_control/gui_library.h>

Gtk::HScale *force_control_p_gainA,
            *force_control_d_gainA,
            *force_control_i_gainA,
            *force_control_p_gainB,
            *force_control_d_gainB,
            *force_control_i_gainB,
            *force_control_spring_deflection;

#endif /* CONTROLLER_GUI_H_ */
