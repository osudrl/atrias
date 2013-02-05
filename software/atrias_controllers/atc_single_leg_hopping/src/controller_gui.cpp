/*
 * controller_gui.cpp
 *
 * atc_single_leg_hopping controller
 *
 *  Created on: Jan 31, 2013
 *      Author: Mikhail Jones
 */

#include <atc_single_leg_hopping/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    gui->get_widget("flight_leg_P_gain_spinbutton", flight_leg_P_gain_spinbutton);
    gui->get_widget("flight_leg_D_gain_spinbutton", flight_leg_D_gain_spinbutton);
    gui->get_widget("flight_hip_P_gain_spinbutton", flight_hip_P_gain_spinbutton);
    gui->get_widget("flight_hip_D_gain_spinbutton", flight_hip_D_gain_spinbutton);
    gui->get_widget("stance_leg_P_gain_spinbutton", stance_leg_P_gain_spinbutton);
    gui->get_widget("stance_leg_D_gain_spinbutton", stance_leg_D_gain_spinbutton);
    gui->get_widget("stance_hip_P_gain_spinbutton", stance_hip_P_gain_spinbutton);
    gui->get_widget("stance_hip_D_gain_spinbutton", stance_hip_D_gain_spinbutton);
    gui->get_widget("slip_height_spinbutton", slip_height_spinbutton);
    gui->get_widget("slip_mass_spinbutton", slip_mass_spinbutton);
    gui->get_widget("slip_spring_spinbutton", slip_spring_spinbutton);
    gui->get_widget("slip_leg_spinbutton", slip_leg_spinbutton);
    gui->get_widget("robot_spring_spinbutton", robot_spring_spinbutton);
    gui->get_widget("debug1_togglebutton", debug1_togglebutton);
    gui->get_widget("debug2_togglebutton", debug2_togglebutton);
    gui->get_widget("debug3_togglebutton", debug3_togglebutton);
    gui->get_widget("debug4_togglebutton", debug4_togglebutton);


    // Set ranges.
    flight_leg_P_gain_spinbutton->set_range(0., 2000.);
    flight_leg_D_gain_spinbutton->set_range(0., 200.);
    flight_hip_P_gain_spinbutton->set_range(0., 2000.);
    flight_hip_D_gain_spinbutton->set_range(0., 200.);
    stance_leg_P_gain_spinbutton->set_range(0., 2000.);
    stance_leg_D_gain_spinbutton->set_range(0., 200.);
    stance_hip_P_gain_spinbutton->set_range(0., 2000.);
    stance_hip_D_gain_spinbutton->set_range(0., 200.);
    slip_height_spinbutton->set_range(0., 0.5);
    slip_mass_spinbutton->set_range(0., 100.);
    slip_spring_spinbutton->set_range(0., 50000.);
    slip_leg_spinbutton->set_range(0.5, 0.90);
    robot_spring_spinbutton->set_range(0., 5000.);

    // Set default values
    flight_leg_P_gain_spinbutton->set_value(100.);
    flight_leg_D_gain_spinbutton->set_value(10.);
    flight_hip_P_gain_spinbutton->set_value(50.);
    flight_hip_D_gain_spinbutton->set_value(2.);
    stance_leg_P_gain_spinbutton->set_value(100.);
    stance_leg_D_gain_spinbutton->set_value(0.);
    stance_hip_P_gain_spinbutton->set_value(50.);
    stance_hip_D_gain_spinbutton->set_value(2.);
    slip_height_spinbutton->set_value(0.05);
    slip_mass_spinbutton->set_value(60.);
    slip_spring_spinbutton->set_value(24500.);
    slip_leg_spinbutton->set_value(0.85);
    robot_spring_spinbutton->set_value(4100.);

    // Set up subscriber and publisher.
    sub = nh.subscribe("atc_single_leg_hopping_status", 0, controllerCallback);
    pub = nh.advertise<atc_single_leg_hopping::controller_input>("atc_single_leg_hopping_input", 0);
    return true;

}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_single_leg_hopping::controller_status &status) {
    controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
    // Get parameters in the atrias_gui namespace.
    nh.getParam("/atrias_gui/flight_leg_P_gain", controllerDataOut.flight_leg_P_gain);
    nh.getParam("/atrias_gui/flight_leg_D_gain", controllerDataOut.flight_leg_D_gain);
    nh.getParam("/atrias_gui/flight_hip_P_gain", controllerDataOut.flight_hip_P_gain);
    nh.getParam("/atrias_gui/flight_hip_D_gain", controllerDataOut.flight_hip_D_gain);
    nh.getParam("/atrias_gui/stance_leg_P_gain", controllerDataOut.stance_leg_P_gain);
    nh.getParam("/atrias_gui/stance_leg_D_gain", controllerDataOut.stance_leg_D_gain);
    nh.getParam("/atrias_gui/stance_hip_P_gain", controllerDataOut.stance_hip_P_gain);
    nh.getParam("/atrias_gui/stance_hip_D_gain", controllerDataOut.stance_hip_D_gain);
    nh.getParam("/atrias_gui/slip_height", controllerDataOut.slip_height);
    nh.getParam("/atrias_gui/slip_mass", controllerDataOut.slip_mass);
    nh.getParam("/atrias_gui/slip_spring", controllerDataOut.slip_spring);
    nh.getParam("/atrias_gui/slip_leg", controllerDataOut.slip_leg);
    nh.getParam("/atrias_gui/robot_spring", controllerDataOut.robot_spring);

    // Configure the GUI.
    flight_leg_P_gain_spinbutton->set_value(controllerDataOut.flight_leg_P_gain);
    flight_leg_D_gain_spinbutton->set_value(controllerDataOut.flight_leg_D_gain);
    flight_hip_P_gain_spinbutton->set_value(controllerDataOut.flight_hip_P_gain);
    flight_hip_D_gain_spinbutton->set_value(controllerDataOut.flight_hip_D_gain);
    stance_leg_P_gain_spinbutton->set_value(controllerDataOut.stance_leg_P_gain);
    stance_leg_D_gain_spinbutton->set_value(controllerDataOut.stance_leg_D_gain);
    stance_hip_P_gain_spinbutton->set_value(controllerDataOut.stance_hip_P_gain);
    stance_hip_D_gain_spinbutton->set_value(controllerDataOut.stance_hip_D_gain);
    slip_height_spinbutton->set_value(controllerDataOut.slip_height);
    slip_mass_spinbutton->set_value(controllerDataOut.slip_mass);
    slip_spring_spinbutton->set_value(controllerDataOut.slip_spring);
    slip_leg_spinbutton->set_value(controllerDataOut.slip_leg);
    robot_spring_spinbutton->set_value(controllerDataOut.robot_spring);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    nh.setParam("/atrias_gui/flight_leg_P_gain", controllerDataOut.flight_leg_P_gain);
    nh.setParam("/atrias_gui/flight_leg_D_gain", controllerDataOut.flight_leg_D_gain);
    nh.setParam("/atrias_gui/flight_hip_P_gain", controllerDataOut.flight_hip_P_gain);
    nh.setParam("/atrias_gui/flight_hip_D_gain", controllerDataOut.flight_hip_D_gain);
    nh.setParam("/atrias_gui/stance_leg_P_gain", controllerDataOut.stance_leg_P_gain);
    nh.setParam("/atrias_gui/stance_leg_D_gain", controllerDataOut.stance_leg_D_gain);
    nh.setParam("/atrias_gui/stance_hip_P_gain", controllerDataOut.stance_hip_P_gain);
    nh.setParam("/atrias_gui/stance_hip_D_gain", controllerDataOut.stance_hip_D_gain);
    nh.setParam("/atrias_gui/slip_height", controllerDataOut.slip_height);
    nh.setParam("/atrias_gui/slip_mass", controllerDataOut.slip_mass);
    nh.setParam("/atrias_gui/slip_spring", controllerDataOut.slip_spring);
    nh.setParam("/atrias_gui/slip_leg", controllerDataOut.slip_leg);
    nh.setParam("/atrias_gui/robot_spring", controllerDataOut.robot_spring);
}

//! \brief Update the GUI.
void guiUpdate() {
    controllerDataOut.flight_leg_P_gain = flight_leg_P_gain_spinbutton->get_value();
    controllerDataOut.flight_leg_D_gain = flight_leg_D_gain_spinbutton->get_value();
    controllerDataOut.flight_hip_P_gain = flight_hip_P_gain_spinbutton->get_value();
    controllerDataOut.flight_hip_D_gain = flight_hip_D_gain_spinbutton->get_value();
    controllerDataOut.stance_leg_P_gain = stance_leg_P_gain_spinbutton->get_value();
    controllerDataOut.stance_leg_D_gain = stance_leg_D_gain_spinbutton->get_value();
    controllerDataOut.stance_hip_P_gain = stance_hip_P_gain_spinbutton->get_value();
    controllerDataOut.stance_hip_D_gain = stance_hip_D_gain_spinbutton->get_value();
    controllerDataOut.slip_height = slip_height_spinbutton->get_value();
    controllerDataOut.slip_mass = slip_mass_spinbutton->get_value();
    controllerDataOut.slip_spring = slip_spring_spinbutton->get_value();
    controllerDataOut.slip_leg = slip_leg_spinbutton->get_value();
    controllerDataOut.robot_spring = robot_spring_spinbutton->get_value();
    controllerDataOut.debug1 = debug1_togglebutton->get_active();
    controllerDataOut.debug2 = debug2_togglebutton->get_active();
    controllerDataOut.debug3 = debug3_togglebutton->get_active();
    controllerDataOut.debug4 = debug4_togglebutton->get_active();

    pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

