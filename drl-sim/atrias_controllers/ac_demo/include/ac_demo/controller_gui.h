/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <ac_demo/controller_common.h>
#include <atrias_control/gui_library.h>

uint8_t demo_ctrl_state;
#define DEMO_CTRL_STOPPED 0
#define DEMO_CTRL_STARTED 1

Gtk::Scale *demo_ctrl_p_scale,
       *demo_ctrl_d_scale,
       *demo_ctrl_amplitude_scale,
       *demo_ctrl_hip_p_scale,
       *demo_ctrl_hip_d_scale;

Gtk::Button *demo_ctrl_start_btn,
        *demo_ctrl_stop_btn;

Gtk::RadioButton *demo_ctrl_controller1_btn,
         *demo_ctrl_controller2_btn,
         *demo_ctrl_controller3_btn,
         *demo_ctrl_controller4_btn,
         *demo_ctrl_controller5_btn,
         *demo_ctrl_controller6_btn;

void start_demo();
void stop_demo();

#endif /* CONTROLLER_GUI_H_ */
