/*
  controller_gui.cpp

  Test Controller

   Created on: June 20, 2012
       Author: Michael Anderson
 */

#include <ac_test/controller_gui.h>

ControllerInitResult guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	ControllerInitResult cir;
    cir.error = true;

    gui->get_widget("StanceP", stance_p_gain);
    gui->get_widget("StanceD", stance_d_gain);
    gui->get_widget("FlightP", flight_p_gain);
    gui->get_widget("FlightD", flight_d_gain);
    gui->get_widget("HipP", hip_p_gain);
    gui->get_widget("HipD", hip_d_gain);
    gui->get_widget("LegLength", desired_leg_length);
    gui->get_widget("LegAngle", leg_angle);
    gui->get_widget("TO_H", takeoff_height);
    gui->get_widget("TD_H", touchdown_height);

    gui->get_widget("status_label", status_label);

    if (stance_p_gain && stance_d_gain
            && flight_p_gain && flight_d_gain
            && hip_p_gain && hip_d_gain
			&& desired_leg_length && leg_angle
            && takeoff_height && touchdown_height
            && status_label) {
        
        stance_p_gain->set_range(0, 10000);
        stance_d_gain->set_range(0, 200);
        flight_p_gain->set_range(0, 10000);
        flight_d_gain->set_range(0, 200);
        hip_p_gain->set_range(0, 10000);
        hip_d_gain->set_range(0, 200);
        desired_leg_length->set_range(.3, 1.3);
        leg_angle->set_range(.5, 2.5);
        takeoff_height->set_range(0, 1.3);
        touchdown_height->set_range(0, 1.3);
        
        stance_p_gain->set_value(5000);
        stance_d_gain->set_value(40);
        flight_p_gain->set_value(100);
        flight_d_gain->set_value(20);
        hip_p_gain->set_value(2000);
        hip_d_gain->set_value(20);
        desired_leg_length->set_value(0.9);
        leg_angle->set_value(1.4);
        takeoff_height->set_value(.89);
        touchdown_height->set_value(.91);
        
        cir.error = false;
    }
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = 0;
    return cir;
}

void guiUpdate(robot_state state, ByteArray controllerStatus, ByteArray &controllerInput) {
    InputData out;

    out.stanceP          = stance_p_gain->get_value();
    out.stanceD          = stance_d_gain->get_value();
    out.flightP          = flight_p_gain->get_value();
    out.flightD          = flight_d_gain->get_value();
    out.hipP             = hip_p_gain->get_value();
    out.hipD             = hip_d_gain->get_value();
    
    out.desiredLegLength = desired_leg_length->get_value();
    out.desiredLegAngle  = leg_angle->get_value();
    out.takeoff_height   = takeoff_height->get_value();
    out.touchdown_height = touchdown_height->get_value();

    // Set the state label.
    if (state.command == CMD_DISABLE)
        status_label->set_label("Disabled");
    else if (state.in_flight)
        status_label->set_label("Flight");
    else
        status_label->set_label("Stance");

    structToByteArray(out, controllerInput);
}

void guiStandby(robot_state state) {
    // Set the state label.
    if (state.command == CMD_DISABLE)
        status_label->set_label("Disabled");
    else if (state.in_flight)
        status_label->set_label("Flight");
    else
        status_label->set_label("Stance");
}

void guiTakedown() {
}
