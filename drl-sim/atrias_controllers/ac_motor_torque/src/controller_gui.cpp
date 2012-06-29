/*
 * controller_gui.cpp
 *
 * Motor Torque Controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <ac_motor_torque/controller_gui.h>

ControllerInitResult guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	ControllerInitResult cir;
    cir.error = true;

    gui->get_widget("torque_A_hscale", torque_A_hscale);
    gui->get_widget("torque_B_hscale", torque_B_hscale);
    gui->get_widget("torque_hip_hscale", torque_hip_hscale);

    if (torque_A_hscale && torque_B_hscale && torque_hip_hscale) {
        torque_A_hscale->set_range(MTR_MIN_TRQ, MTR_MAX_TRQ);
        torque_B_hscale->set_range(MTR_MIN_TRQ, MTR_MAX_TRQ);
        torque_hip_hscale->set_range(MTR_MIN_TRQ, MTR_MAX_TRQ);
        cir.error = false;
    }
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = 0;
    return cir;
}

void guiUpdate(robot_state state, ByteArray controllerStatus, ByteArray &controllerInput) {
    InputData out;

    out.mtr_trqA = torque_A_hscale->get_value();
    out.mtr_trqB = torque_B_hscale->get_value();
    out.mtr_trq_hip = torque_hip_hscale->get_value();

    structToByteArray(out, controllerInput);
}

void guiStandby(robot_state state) {

}

void guiTakedown() {

}
