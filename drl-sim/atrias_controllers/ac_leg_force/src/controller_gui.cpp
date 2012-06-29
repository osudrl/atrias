/*
 * controller_gui.cpp
 *
 * Leg Force Controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <ac_leg_force/controller_gui.h>

ControllerInitResult guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	ControllerInitResult cir;
    cir.error = true;
    gui->get_widget("force_control_p_gainA", force_control_p_gainA);
    gui->get_widget("force_control_d_gainA", force_control_d_gainA);
    gui->get_widget("force_control_i_gainA", force_control_i_gainA);
    gui->get_widget("force_control_p_gainB", force_control_p_gainB);
    gui->get_widget("force_control_d_gainB", force_control_d_gainB);
    gui->get_widget("force_control_i_gainB", force_control_i_gainB);
    gui->get_widget("force_control_spring_deflection", force_control_spring_deflection);

    if (force_control_p_gainA && force_control_d_gainA && force_control_p_gainA &&
    		force_control_p_gainB && force_control_d_gainB && force_control_p_gainB &&
    		force_control_spring_deflection) {
        force_control_p_gainA->set_range(0.0,5000.0);
        force_control_d_gainA->set_range(0.0,50.0);
        force_control_i_gainA->set_range(30.0,960.0);
        force_control_p_gainB->set_range(10.0,500.0);
        force_control_d_gainB->set_range(0.0,10.0);
        force_control_i_gainB->set_range(30.0,960.0);
        force_control_spring_deflection->set_range(0.0,50.0);
        cir.error = false;
    }
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = 0;
    return cir;
}

void guiUpdate(robot_state state, ByteArray controllerStatus, ByteArray &controllerInput) {
    InputData out;

    out.p_gainA = force_control_p_gainA->get_value();
	out.d_gainA = force_control_d_gainA->get_value();
	out.i_gainA = force_control_i_gainA->get_value();
    out.p_gainB = force_control_p_gainB->get_value();
	out.d_gainB = force_control_d_gainB->get_value();
	out.i_gainB = force_control_i_gainB->get_value();
	out.spring_deflection = force_control_spring_deflection->get_value();

    structToByteArray(out, controllerInput);
}

void guiStandby(robot_state state) {

}

void guiTakedown() {

}
