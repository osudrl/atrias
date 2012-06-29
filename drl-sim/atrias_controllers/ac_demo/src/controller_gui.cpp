/*
 * controller_gui.cpp
 *
 * Demo Controller
 *
 *  Created on: May 6, 2012
 *      Author: Michael Anderson
 */

#include <ac_demo/controller_gui.h>

ControllerInitResult guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	ControllerInitResult cir;
    cir.error = true;

    gui->get_widget("p_scale", demo_ctrl_p_scale);
    gui->get_widget("d_scale", demo_ctrl_d_scale);
    gui->get_widget("hip_p_scale", demo_ctrl_hip_p_scale);
    gui->get_widget("hip_d_scale", demo_ctrl_hip_d_scale);
    gui->get_widget("amplitude_scale", demo_ctrl_amplitude_scale);
    gui->get_widget("start_btn", demo_ctrl_start_btn);
    gui->get_widget("stop_btn", demo_ctrl_stop_btn);
    gui->get_widget("controller1_btn", demo_ctrl_controller1_btn);
    gui->get_widget("controller2_btn", demo_ctrl_controller2_btn);
    gui->get_widget("controller3_btn", demo_ctrl_controller3_btn);
    gui->get_widget("controller4_btn", demo_ctrl_controller4_btn);
    gui->get_widget("controller5_btn", demo_ctrl_controller5_btn);
    gui->get_widget("controller6_btn", demo_ctrl_controller6_btn);
    demo_ctrl_state = DEMO_CTRL_STOPPED;

    if (demo_ctrl_p_scale && demo_ctrl_d_scale && demo_ctrl_hip_p_scale
            && demo_ctrl_hip_d_scale && demo_ctrl_amplitude_scale && demo_ctrl_start_btn
            && demo_ctrl_stop_btn && demo_ctrl_controller1_btn && demo_ctrl_controller2_btn
            && demo_ctrl_controller3_btn && demo_ctrl_controller4_btn
			&& demo_ctrl_controller5_btn && demo_ctrl_controller6_btn) {
        demo_ctrl_p_scale->set_range(0.0,5000.0);
        demo_ctrl_d_scale->set_range(0.0,60.0);
        demo_ctrl_hip_p_scale->set_range(0.0,3000.0);
        demo_ctrl_hip_d_scale->set_range(0.0,50.0);
        demo_ctrl_amplitude_scale->set_range(0.0,3.0);

        demo_ctrl_start_btn->signal_clicked().connect(sigc::ptr_fun(start_demo));
        demo_ctrl_stop_btn->signal_clicked().connect(sigc::ptr_fun(stop_demo));
        cir.error = false;
    }
    cir.controllerInputSize = sizeof(InputData);
    cir.controllerStatusSize = 0;
    return cir;
}

void guiUpdate(robot_state state, ByteArray controllerStatus, ByteArray &controllerInput) {
    InputData out;

    out.commandedState = demo_ctrl_state;
    if (demo_ctrl_controller1_btn->get_active())
            out.commandedDemo = 0;
    else if (demo_ctrl_controller2_btn->get_active())
            out.commandedDemo = 1;
    else if (demo_ctrl_controller3_btn->get_active())
            out.commandedDemo = 2;
    else if (demo_ctrl_controller4_btn->get_active())
            out.commandedDemo = 3;
    else if (demo_ctrl_controller5_btn->get_active())
            out.commandedDemo = 4;
    else if (demo_ctrl_controller6_btn->get_active())
            out.commandedDemo = 5;

    out.amplitude = demo_ctrl_amplitude_scale->get_value();
    out.p_gain = demo_ctrl_p_scale->get_value();
    out.d_gain = demo_ctrl_d_scale->get_value();
    out.hip_p_gain = demo_ctrl_hip_p_scale->get_value();
    out.hip_d_gain = demo_ctrl_hip_d_scale->get_value();

    structToByteArray(out, controllerInput);
}

void guiStandby(robot_state state) {

}

void guiTakedown() {

}

void start_demo(void) {
    demo_ctrl_state = DEMO_CTRL_STARTED;
}

void stop_demo(void) {
    demo_ctrl_state = DEMO_CTRL_STOPPED;
}
