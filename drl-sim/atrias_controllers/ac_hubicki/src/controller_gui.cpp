/*
 * controller_gui.cpp
 *
 * Hubicki Controller
 *
 *  Created on: May 6, 2012
 *      Author: Michael Anderson
 */

#include <ac_hubicki/controller_gui.h>

ControllerInitResult guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	ControllerInitResult cir;
	cir.error = true;
    gui->get_widget("state_label", hubicki_state_label);
    gui->get_widget("desired_velocity_spinbutton", hubicki_desired_velocity_spinbutton);
    gui->get_widget("hor_vel_gain_spinbutton", hubicki_hor_vel_gain_spinbutton);
    gui->get_widget("leg_angle_gain_spinbutton", hubicki_leg_angle_gain_spinbutton);
    gui->get_widget("stance_p_gain_spinbutton", hubicki_stance_p_gain_spinbutton);
    gui->get_widget("stance_d_gain_spinbutton", hubicki_stance_d_gain_spinbutton);
    gui->get_widget("stance_spring_threshold_spinbutton", hubicki_stance_spring_threshold_spinbutton);
    gui->get_widget("desired_height_spinbutton", hubicki_desired_height_spinbutton);
    gui->get_widget("leg_force_gain_spinbutton", hubicki_leg_force_gain_spinbutton);
    gui->get_widget("preferred_leg_len_spinbutton", hubicki_preferred_leg_len_spinbutton);
    gui->get_widget("flight_p_gain_spinbutton", hubicki_flight_p_gain_spinbutton);
    gui->get_widget("flight_d_gain_spinbutton", hubicki_flight_d_gain_spinbutton);
    gui->get_widget("flight_spring_threshold_spinbutton", hubicki_flight_spring_threshold_spinbutton);
    gui->get_widget("stance_hip_p_gain", hubicki_stance_hip_p_gain);
    gui->get_widget("stance_hip_d_gain", hubicki_stance_hip_d_gain);
    gui->get_widget("flight_hip_p_gain", hubicki_flight_hip_p_gain);
    gui->get_widget("flight_hip_d_gain", hubicki_flight_hip_d_gain);

    if (hubicki_desired_velocity_spinbutton && hubicki_hor_vel_gain_spinbutton
            && hubicki_leg_angle_gain_spinbutton && hubicki_stance_p_gain_spinbutton
            && hubicki_stance_d_gain_spinbutton && hubicki_stance_spring_threshold_spinbutton
            && hubicki_desired_height_spinbutton && hubicki_leg_force_gain_spinbutton
            && hubicki_preferred_leg_len_spinbutton && hubicki_flight_p_gain_spinbutton
            && hubicki_flight_d_gain_spinbutton && hubicki_flight_spring_threshold_spinbutton
            && hubicki_stance_hip_p_gain && hubicki_stance_hip_d_gain
            && hubicki_flight_hip_p_gain && hubicki_flight_hip_d_gain) {
        hubicki_state_label->set_label("Initializing");

        hubicki_desired_velocity_spinbutton->set_range(-5., 5.);
        hubicki_hor_vel_gain_spinbutton->set_range(0., 10.);
        hubicki_leg_angle_gain_spinbutton->set_range(0., 200.);
        hubicki_stance_p_gain_spinbutton->set_range(0., 6000.);
        hubicki_stance_d_gain_spinbutton->set_range(0., 150.);
        hubicki_stance_spring_threshold_spinbutton->set_range(0., 0.4);
        hubicki_desired_height_spinbutton->set_range(0., 3.);
        hubicki_leg_force_gain_spinbutton->set_range(0., 6000.);
        hubicki_preferred_leg_len_spinbutton->set_range(0.7, 1.);
        hubicki_flight_p_gain_spinbutton->set_range(0., 4000.);
        hubicki_flight_d_gain_spinbutton->set_range(0., 150.);
        hubicki_flight_spring_threshold_spinbutton->set_range(0., 0.4);

        hubicki_stance_hip_p_gain->set_range(0,5000.0);
        hubicki_stance_hip_d_gain->set_range(0,100.0);
        hubicki_flight_hip_p_gain->set_range(0,5000.0);
        hubicki_flight_hip_d_gain->set_range(0,100.0);
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
        hubicki_state_label->set_label("Disabled");
    else if (cs->in_flight)
        hubicki_state_label->set_label("Flight");
    else
        hubicki_state_label->set_label("Stance");

    InputData out;

    out.des_hor_vel =               hubicki_desired_velocity_spinbutton->get_value();
    out.des_hop_ht =                hubicki_desired_height_spinbutton->get_value();
    out.hor_vel_gain =              hubicki_hor_vel_gain_spinbutton->get_value();
    out.hop_ht_gain =               hubicki_leg_force_gain_spinbutton->get_value();
    out.leg_ang_gain =              hubicki_leg_angle_gain_spinbutton->get_value();
    out.stance_p_gain =             hubicki_stance_p_gain_spinbutton->get_value();
    out.stance_d_gain =             hubicki_stance_d_gain_spinbutton->get_value();
    out.stance_spring_threshold =   hubicki_stance_spring_threshold_spinbutton->get_value();
    out.preferred_leg_len =         hubicki_preferred_leg_len_spinbutton->get_value();
    out.flight_p_gain =             hubicki_flight_p_gain_spinbutton->get_value();
    out.flight_d_gain =             hubicki_flight_d_gain_spinbutton->get_value();
    out.flight_spring_threshold =   hubicki_flight_spring_threshold_spinbutton->get_value();
    out.stance_hip_p_gain =         hubicki_stance_hip_p_gain->get_value();
    out.stance_hip_d_gain =         hubicki_stance_hip_d_gain->get_value();
    out.flight_hip_p_gain =         hubicki_flight_hip_p_gain->get_value();
    out.flight_hip_d_gain =         hubicki_flight_hip_d_gain->get_value();

    structToByteArray(out, controllerInput);
}

void guiStandby(robot_state state) {
	hubicki_state_label->set_label("Disabled");
}

void guiTakedown() {

}
