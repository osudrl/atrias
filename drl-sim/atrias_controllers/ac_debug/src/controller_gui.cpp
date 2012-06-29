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

    gui->get_widget("command_label", command_label);
    gui->get_widget("loop_time_label", loop_time_label);
    gui->get_widget("torque_A_label", torque_A_label);
    gui->get_widget("torque_B_label", torque_B_label);
    gui->get_widget("subtract_A_torque_button", subtract_A_torque_button);
    gui->get_widget("add_A_torque_button", add_A_torque_button);
    gui->get_widget("subtract_B_torque_button", subtract_B_torque_button);
    gui->get_widget("add_B_torque_button", add_B_torque_button);
    gui->get_widget("progress_bar", progress_bar);
    gui->get_widget("speed_hscale", speed_hscale);

    if (command_label && loop_time_label && torque_A_label && torque_B_label &&
    		subtract_A_torque_button && add_A_torque_button &&
    		subtract_B_torque_button && add_B_torque_button && progress_bar &&
    		speed_hscale) {
        speed_hscale->set_range(0., 1.);
        add_A_torque_button->signal_clicked().connect(sigc::ptr_fun(add_a_torque));
        subtract_A_torque_button->signal_clicked().connect(sigc::ptr_fun(subtract_a_torque));
        add_B_torque_button->signal_clicked().connect(sigc::ptr_fun(add_b_torque));
        subtract_B_torque_button->signal_clicked().connect(sigc::ptr_fun(subtract_b_torque));
        cir.error = false;
    }
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = sizeof(ControllerStatus);
    return cir;
}

void guiUpdate(robot_state state, ByteArray controllerStatus, ByteArray &controllerInput) {
    ControllerStatus *cs = BYTE_ARRAY_TO_STRUCT(controllerStatus, ControllerStatus*);
    InputData out;

    out.mtr_trqA = torqueA;
    out.mtr_trqB = torqueB;

    progress_bar->set_fraction(cs->progress);

    char buffer[20];
    sprintf(buffer, "%i", (int)cs->loopTime);
    loop_time_label->set_label(buffer);

    sprintf(buffer, "%i", (int)state.motor_torqueA);
    torque_A_label->set_label(buffer);
    sprintf(buffer, "%i", (int)state.motor_torqueB);
    torque_B_label->set_label(buffer);

    switch (state.command) {
    	case CMD_DISABLE: {
    		command_label->set_label("DISABLE");
    		break;
    	}
    	case CMD_RUN: {
    		command_label->set_label("RUN");
    		break;
    	}
    	case CMD_RESTART: {
    		command_label->set_label("RESTART");
    		break;
    	}
    	default: {
    		command_label->set_label("UNKNOWN");
    		break;
    	}
    }

    out.adjustment_speed = speed_hscale->get_value();

    structToByteArray(out, controllerInput);
}

void add_a_torque() {
	torqueA++;
}

void subtract_a_torque() {
	torqueA--;
}

void add_b_torque() {
	torqueB++;
}

void subtract_b_torque() {
	torqueB--;
}

void guiStandby(robot_state state) {
    switch (state.command) {
    	case CMD_DISABLE: {
    		command_label->set_label("DISABLE");
    		break;
    	}
    	case CMD_RUN: {
    		command_label->set_label("RUN");
    		break;
    	}
    	case CMD_RESTART: {
    		command_label->set_label("RESTART");
    		break;
    	}
    	default: {
    		command_label->set_label("UNKNOWN");
    		break;
    	}
    }
}

void guiTakedown() {

}
