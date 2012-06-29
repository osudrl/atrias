/*
 * controller_gui.cpp
 *
 * Motor Position Controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <ac_motor_position/controller_gui.h>

ControllerInitResult guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	ControllerInitResult cir;
    cir.error = true;

    gui->get_widget("position_A_hscale", position_A_hscale);
    gui->get_widget("position_B_hscale", position_B_hscale);
    gui->get_widget("position_p_hscale", p_hscale);
    gui->get_widget("position_d_hscale", d_hscale);

    gui->get_widget("set_position_checkbutton", set_position_checkbutton);

    if (position_A_hscale && position_B_hscale && p_hscale && d_hscale &&
        set_position_checkbutton) {
        position_A_hscale->set_range(-2.3562, 0.3054);
        position_B_hscale->set_range(2.8362, 5.4978);
        p_hscale->set_range(0., 50000.);
        d_hscale->set_range(0., 5000.);
        cir.error = false;
    }
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = 0;
    return cir;
}

void guiUpdate(robot_state state, ByteArray controllerStatus, ByteArray &controllerInput) {
    InputData out;

    out.mtr_angA = position_A_hscale->get_value();
    out.mtr_angB = position_B_hscale->get_value();
    out.p_gain = p_hscale->get_value();
    out.d_gain = d_hscale->get_value();

    structToByteArray(out, controllerInput);
}

void guiStandby(robot_state state) {
    if (set_position_checkbutton->get_active()) {
        position_A_hscale->set_value(state.motor_angleA);
        position_B_hscale->set_value(state.motor_angleB);
    }
}

void guiTakedown() {

}
