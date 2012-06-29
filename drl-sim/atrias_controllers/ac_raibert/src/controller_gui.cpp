/*
 * controller_gui.cpp
 *
 * FORCE CONTROL FOR VERTICAL HOPPING - ATRIAS 2.0 - By Mikhail Jones
 * Code is originally from Devin Koepl's Raibert controller for ATRIAS 1.0
 *
 * Ported to new controller system by Michael Anderson
 *
 *  Created on: May 6, 2012
 *      Author: Michael Anderson
 */

#include <ac_raibert/controller_gui.h>

ControllerInitResult guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	ControllerInitResult cir;
    cir.error = true;

    gui->get_widget("state_label", raibert_state_label);
    gui->get_widget("desired_velocity_spinbutton", raibert_desired_velocity_spinbutton);
    gui->get_widget("hor_vel_gain_spinbutton", raibert_hor_vel_gain_spinbutton);
    gui->get_widget("leg_angle_gain_spinbutton", raibert_leg_angle_gain_spinbutton);
    gui->get_widget("stance_p_gain_spinbutton", raibert_stance_p_gain_spinbutton);
    gui->get_widget("stance_d_gain_spinbutton", raibert_stance_d_gain_spinbutton);
    gui->get_widget("stance_spring_threshold_spinbutton", raibert_stance_spring_threshold_spinbutton);
    gui->get_widget("desired_height_spinbutton", raibert_desired_height_spinbutton);
    gui->get_widget("leg_force_gain_spinbutton", raibert_leg_force_gain_spinbutton);
    gui->get_widget("preferred_leg_len_spinbutton", raibert_preferred_leg_len_spinbutton);
    gui->get_widget("flight_p_gain_spinbutton", raibert_flight_p_gain_spinbutton);
    gui->get_widget("flight_d_gain_spinbutton", raibert_flight_d_gain_spinbutton);
    gui->get_widget("flight_spring_threshold_spinbutton", raibert_flight_spring_threshold_spinbutton);
    gui->get_widget("stance_hip_p_gain", raibert_stance_hip_p_gain);
    gui->get_widget("stance_hip_d_gain", raibert_stance_hip_d_gain);
    gui->get_widget("flight_hip_p_gain", raibert_flight_hip_p_gain);
    gui->get_widget("flight_hip_d_gain", raibert_flight_hip_d_gain);

    if (raibert_desired_velocity_spinbutton && raibert_hor_vel_gain_spinbutton
            && raibert_leg_angle_gain_spinbutton && raibert_stance_p_gain_spinbutton
            && raibert_stance_d_gain_spinbutton && raibert_stance_spring_threshold_spinbutton
            && raibert_desired_height_spinbutton && raibert_leg_force_gain_spinbutton
            && raibert_preferred_leg_len_spinbutton && raibert_flight_p_gain_spinbutton
            && raibert_flight_d_gain_spinbutton && raibert_flight_spring_threshold_spinbutton
            && raibert_stance_hip_p_gain && raibert_stance_hip_d_gain
            && raibert_flight_hip_p_gain && raibert_flight_hip_d_gain) {
        raibert_state_label->set_label("Initializing");

        raibert_desired_velocity_spinbutton->set_range(-5., 5.);
        raibert_hor_vel_gain_spinbutton->set_range(-10., 10.);
        raibert_leg_angle_gain_spinbutton->set_range(0., 5000.);
        raibert_stance_p_gain_spinbutton->set_range(0., 6000.);
        raibert_stance_d_gain_spinbutton->set_range(0., 150.);
        raibert_stance_spring_threshold_spinbutton->set_range(0., 0.4);
        raibert_desired_height_spinbutton->set_range(0., 10.);
        raibert_leg_force_gain_spinbutton->set_range(0., 50000.);
        raibert_preferred_leg_len_spinbutton->set_range(0.7, 1.);
        raibert_flight_p_gain_spinbutton->set_range(0., 4000.);
        raibert_flight_d_gain_spinbutton->set_range(0., 150.);
        raibert_flight_spring_threshold_spinbutton->set_range(0., 0.4);

        raibert_stance_hip_p_gain->set_range(0,5000.0);
        raibert_stance_hip_d_gain->set_range(0,100.0);
        raibert_flight_hip_p_gain->set_range(0,5000.0);
        raibert_flight_hip_d_gain->set_range(0,100.0);
        cir.error = false;
    }
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = sizeof(ControllerStatus);
    return cir;
}

void guiUpdate(robot_state state, ByteArray controllerStatus, ByteArray &controllerInput) {
    ControllerStatus *cs = BYTE_ARRAY_TO_STRUCT(controllerStatus, ControllerStatus*);

    // Set the state label.
    if (state.command == CMD_DISABLE)
        raibert_state_label->set_label("Disabled");
    else if (cs->in_flight)
        raibert_state_label->set_label("Flight");
    else
        raibert_state_label->set_label("Stance");

    InputData out;

    out.des_hor_vel =               raibert_desired_velocity_spinbutton->get_value();
    out.des_hop_ht =                raibert_desired_height_spinbutton->get_value();
    out.hor_vel_gain =              raibert_hor_vel_gain_spinbutton->get_value();
    out.hop_ht_gain =               raibert_leg_force_gain_spinbutton->get_value();
    out.leg_ang_gain =              raibert_leg_angle_gain_spinbutton->get_value();
    out.stance_p_gain =             raibert_stance_p_gain_spinbutton->get_value();
    out.stance_d_gain =             raibert_stance_d_gain_spinbutton->get_value();
    out.stance_spring_threshold =   raibert_stance_spring_threshold_spinbutton->get_value();
    out.preferred_leg_len =         raibert_preferred_leg_len_spinbutton->get_value();
    out.flight_p_gain =             raibert_flight_p_gain_spinbutton->get_value();
    out.flight_d_gain =             raibert_flight_d_gain_spinbutton->get_value();
    out.flight_spring_threshold =   raibert_flight_spring_threshold_spinbutton->get_value();
    out.stance_hip_p_gain =         raibert_stance_hip_p_gain->get_value();
    out.stance_hip_d_gain =         raibert_stance_hip_d_gain->get_value();
    out.flight_hip_p_gain =         raibert_flight_hip_p_gain->get_value();
    out.flight_hip_d_gain =         raibert_flight_hip_d_gain->get_value();

    structToByteArray(out, controllerInput);
}

void guiStandby(robot_state state) {
	raibert_state_label->set_label("Disabled");
}

void guiTakedown() {

}
