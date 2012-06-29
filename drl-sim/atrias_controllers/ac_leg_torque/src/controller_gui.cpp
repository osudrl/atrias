/*
 * controller_gui.cpp
 *
 * Leg Torque Controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <ac_leg_torque/controller_gui.h>

ControllerInitResult guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	ControllerInitResult cir;
    cir.error = true;

    gui->get_widget("length_torque_hscale", length_torque_hscale);
    gui->get_widget("angle_torque_hscale", angle_torque_hscale);

    if (length_torque_hscale && angle_torque_hscale) {
        length_torque_hscale->set_range(-10., 10.);
        angle_torque_hscale->set_range(-10., 10.);
        cir.error = false;
    }
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = 0;
    return cir;
}

void guiUpdate(robot_state state, ByteArray controllerStatus, ByteArray &controllerInput) {
    InputData out;

    out.leg_ang_trq = angle_torque_hscale->get_value();
    out.leg_len_trq = length_torque_hscale->get_value();

    structToByteArray(out, controllerInput);
}

void guiStandby(robot_state state) {

}

void guiTakedown() {

}
