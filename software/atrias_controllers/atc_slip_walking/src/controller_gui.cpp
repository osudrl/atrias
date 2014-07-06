/**
 * @file controller_gui.cpp
 * @author Mikhail Jones
 */

#include <atc_slip_walking/controller_gui.h>

void flight_td_pressed()  {controllerDataOut.flight_td = 1;}
void flight_td_released() {controllerDataOut.flight_td = 0;}
void flight_to_pressed()  {controllerDataOut.flight_to = 1;}
void flight_to_released() {controllerDataOut.flight_to = 0;}

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    // Main controller options
    gui->get_widget("main_controller_combobox", main_controller_combobox);
    gui->get_widget("switch_method_combobox", switch_method_combobox);

    // Gait options
    gui->get_widget("swing_leg_retraction_spinbutton", swing_leg_retraction_spinbutton);
    swing_leg_retraction_spinbutton->set_range(0.0, 0.35);
    swing_leg_retraction_spinbutton->set_increments(0.01, 0.0);
    swing_leg_retraction_spinbutton->set_value(0.15);
    gui->get_widget("stance_leg_extension_spinbutton", stance_leg_extension_spinbutton);
    stance_leg_extension_spinbutton->set_range(0.0, 0.05);
    stance_leg_extension_spinbutton->set_increments(0.01, 0.0);
    stance_leg_extension_spinbutton->set_value(0.0);
    gui->get_widget("leg_length_spinbutton", leg_length_spinbutton);
    leg_length_spinbutton->set_range(0.65, 0.95);
    leg_length_spinbutton->set_increments(0.01, 0.0);
    leg_length_spinbutton->set_value(0.9);
    gui->get_widget("torso_angle_spinbutton", torso_angle_spinbutton);
    torso_angle_spinbutton->set_range(M_PI, 2.0*M_PI);
    torso_angle_spinbutton->set_increments(0.01, 0.0);
    torso_angle_spinbutton->set_value(3.0*M_PI/2.0);
    gui->get_widget("q1_spinbutton", q1_spinbutton);
    q1_spinbutton->set_range(0.0, M_PI);
    q1_spinbutton->set_increments(0.01, 0.0);
    q1_spinbutton->set_value(1.144);
    gui->get_widget("q2_spinbutton", q2_spinbutton);
    q2_spinbutton->set_range(0.0, M_PI);
    q2_spinbutton->set_increments(0.01, 0.0);
    q2_spinbutton->set_value(1.297);
    gui->get_widget("q3_spinbutton", q3_spinbutton);
    q3_spinbutton->set_range(0.0, M_PI);
    q3_spinbutton->set_increments(0.01, 0.0);
    q3_spinbutton->set_value(1.845);
    gui->get_widget("q4_spinbutton", q4_spinbutton);
    q4_spinbutton->set_range(0.0, M_PI);
    q4_spinbutton->set_increments(0.01, 0.0);
    q4_spinbutton->set_value(1.998);

    // Leg gains
    gui->get_widget("leg_pos_kp_spinbutton", leg_pos_kp_spinbutton);
    leg_pos_kp_spinbutton->set_range(0.0, 1000.0);
    leg_pos_kp_spinbutton->set_increments(10.0, 0.0);
    leg_pos_kp_spinbutton->set_value(500.0);
    gui->get_widget("leg_pos_kd_spinbutton", leg_pos_kd_spinbutton);
    leg_pos_kd_spinbutton->set_range(0.0, 100.0);
    leg_pos_kd_spinbutton->set_increments(1.0, 0.0);
    leg_pos_kd_spinbutton->set_value(50.0);
    gui->get_widget("leg_for_kp_spinbutton", leg_for_kp_spinbutton);
    leg_for_kp_spinbutton->set_range(0.0, 500.0);
    leg_for_kp_spinbutton->set_increments(10.0, 0.0);
    leg_for_kp_spinbutton->set_value(100.0);
    gui->get_widget("leg_for_ki_spinbutton", leg_for_ki_spinbutton);
    leg_for_ki_spinbutton->set_range(0.0, 500.0);
    leg_for_ki_spinbutton->set_increments(10.0, 0.0);
    leg_for_ki_spinbutton->set_value(0.0);
    gui->get_widget("leg_for_kd_spinbutton", leg_for_kd_spinbutton);
    leg_for_kd_spinbutton->set_range(0.0, 100.0);
    leg_for_kd_spinbutton->set_increments(1.0, 0.0);
    leg_for_kd_spinbutton->set_value(10.0);

    // Hip gains
    gui->get_widget("hip_pos_kp_spinbutton", hip_pos_kp_spinbutton);
    hip_pos_kp_spinbutton->set_range(0.0, 300.0);
    hip_pos_kp_spinbutton->set_increments(10.0, 0.0);
    hip_pos_kp_spinbutton->set_value(150.0);
    gui->get_widget("hip_pos_kd_spinbutton", hip_pos_kd_spinbutton);
    hip_pos_kd_spinbutton->set_range(0.0, 50.0);
    hip_pos_kd_spinbutton->set_increments(1.0, 0.0);
    hip_pos_kd_spinbutton->set_value(10.0);
    gui->get_widget("left_toe_pos_spinbutton", left_toe_pos_spinbutton);
    left_toe_pos_spinbutton->set_range(2.1, 2.5);
    left_toe_pos_spinbutton->set_increments(0.01, 0.0);
    left_toe_pos_spinbutton->set_value(2.17);
    gui->get_widget("right_toe_pos_spinbutton", right_toe_pos_spinbutton);
    right_toe_pos_spinbutton->set_range(2.1, 2.55);
    right_toe_pos_spinbutton->set_increments(0.01, 0.0);
    right_toe_pos_spinbutton->set_value(2.5);

    // Torso options
    gui->get_widget("qvpp_spinbutton", qvpp_spinbutton);
    qvpp_spinbutton->set_range(-M_PI/2.0, M_PI/2.0);
    qvpp_spinbutton->set_increments(0.01, 0.0);
    qvpp_spinbutton->set_value(-0.25);
    gui->get_widget("rvpp_spinbutton", rvpp_spinbutton);
    rvpp_spinbutton->set_range(-0.5, 2.0);
    rvpp_spinbutton->set_increments(0.01, 0.0);
    rvpp_spinbutton->set_value(0.4);

    // Debug
    gui->get_widget("current_limit_spinbutton", current_limit_spinbutton);
    current_limit_spinbutton->set_range(0.0, 60.0);
    current_limit_spinbutton->set_increments(5.0, 0.0);
    current_limit_spinbutton->set_value(60.0);
    gui->get_widget("velocity_limit_spinbutton", velocity_limit_spinbutton);
    velocity_limit_spinbutton->set_range(0.0, 15.0);
    velocity_limit_spinbutton->set_increments(0.5, 0.0);
    velocity_limit_spinbutton->set_value(12.0);
    gui->get_widget("deflection_limit_spinbutton", deflection_limit_spinbutton);
    deflection_limit_spinbutton->set_range(0.0, 0.3);
    deflection_limit_spinbutton->set_increments(0.05, 0.0);
    deflection_limit_spinbutton->set_value(0.25);
    gui->get_widget("walking_state_spinbutton", walking_state_spinbutton);
    walking_state_spinbutton->set_range(0, 3);
    walking_state_spinbutton->set_increments(1, 0);
    walking_state_spinbutton->set_value(0);
    gui->get_widget("flight_td_button", flight_td_button);
    flight_td_button->signal_pressed().connect(sigc::ptr_fun((void(*)())flight_td_pressed));
    flight_td_button->signal_released().connect(sigc::ptr_fun((void(*)())flight_td_released));
    gui->get_widget("flight_to_button", flight_to_button);
    flight_to_button->signal_pressed().connect(sigc::ptr_fun((void(*)())flight_to_pressed));
    flight_to_button->signal_released().connect(sigc::ptr_fun((void(*)())flight_to_released));

    // Set up subscriber and publisher.
    sub = nh.subscribe("ATCSlipWalking_status", 0, controllerCallback);
    pub = nh.advertise<atc_slip_walking::controller_input>("ATCSlipWalking_input", 0);
    return true;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_slip_walking::controller_status &status) {
    controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
    // Main controller options
    int main_controller;
    nh.getParam("/atrias_gui/main_controller", main_controller);
    controllerDataOut.main_controller = (uint8_t)main_controller;
    main_controller_combobox->set_active(controllerDataOut.main_controller);
    int switch_method;
    nh.getParam("/atrias_gui/switch_method", switch_method);
    controllerDataOut.switch_method = (uint8_t)switch_method;
    switch_method_combobox->set_active(controllerDataOut.switch_method);

    // Gait options
    nh.getParam("/atrias_gui/swing_leg_retraction", controllerDataOut.swing_leg_retraction);
    swing_leg_retraction_spinbutton->set_value(controllerDataOut.swing_leg_retraction);
    nh.getParam("/atrias_gui/stance_leg_extension", controllerDataOut.stance_leg_extension);
    stance_leg_extension_spinbutton->set_value(controllerDataOut.stance_leg_extension);
    nh.getParam("/atrias_gui/leg_length", controllerDataOut.leg_length);
    leg_length_spinbutton->set_value(controllerDataOut.leg_length);
    nh.getParam("/atrias_gui/torso_angle", controllerDataOut.torso_angle);
    torso_angle_spinbutton->set_value(controllerDataOut.torso_angle);
    nh.getParam("/atrias_gui/q1", controllerDataOut.q1);
    q1_spinbutton->set_value(controllerDataOut.q1);
    nh.getParam("/atrias_gui/q2", controllerDataOut.q2);
    q2_spinbutton->set_value(controllerDataOut.q2);
    nh.getParam("/atrias_gui/q3", controllerDataOut.q3);
    q3_spinbutton->set_value(controllerDataOut.q3);
    nh.getParam("/atrias_gui/q4", controllerDataOut.q4);
    q4_spinbutton->set_value(controllerDataOut.q4);

    // Leg gains
    nh.getParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
    leg_pos_kp_spinbutton->set_value(controllerDataOut.leg_pos_kp);
    nh.getParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
    leg_pos_kd_spinbutton->set_value(controllerDataOut.leg_pos_kd);
    nh.getParam("/atrias_gui/leg_for_kp", controllerDataOut.leg_for_kp);
    leg_for_kp_spinbutton->set_value(controllerDataOut.leg_for_kp);
    nh.getParam("/atrias_gui/leg_for_ki", controllerDataOut.leg_for_ki);
    leg_for_ki_spinbutton->set_value(controllerDataOut.leg_for_ki);
    nh.getParam("/atrias_gui/leg_for_kd", controllerDataOut.leg_for_kd);
    leg_for_kd_spinbutton->set_value(controllerDataOut.leg_for_kd);

    // Hip gains
    nh.getParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
    hip_pos_kp_spinbutton->set_value(controllerDataOut.hip_pos_kp);
    nh.getParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
    hip_pos_kd_spinbutton->set_value(controllerDataOut.hip_pos_kd);
    nh.getParam("/atrias_gui/left_toe_pos", controllerDataOut.left_toe_pos);
    left_toe_pos_spinbutton->set_value(controllerDataOut.left_toe_pos);
    nh.getParam("/atrias_gui/right_toe_pos", controllerDataOut.right_toe_pos);
    right_toe_pos_spinbutton->set_value(controllerDataOut.right_toe_pos);

    // Torso options
    nh.getParam("/atrias_gui/qvpp", controllerDataOut.qvpp);
    qvpp_spinbutton->set_value(controllerDataOut.qvpp);
    nh.getParam("/atrias_gui/rvpp", controllerDataOut.rvpp);
    rvpp_spinbutton->set_value(controllerDataOut.rvpp);

    // Debug
    nh.getParam("/atrias_gui/current_limit", controllerDataOut.current_limit);
    current_limit_spinbutton->set_value(controllerDataOut.current_limit);
    nh.getParam("/atrias_gui/velocity_limit", controllerDataOut.velocity_limit);
    velocity_limit_spinbutton->set_value(controllerDataOut.velocity_limit);
    nh.getParam("/atrias_gui/deflection_limit", controllerDataOut.deflection_limit);
    deflection_limit_spinbutton->set_value(controllerDataOut.deflection_limit);
    nh.getParam("/atrias_gui/walking_state", controllerDataOut.walking_state);
    walking_state_spinbutton->set_value(controllerDataOut.walking_state);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    // Main controller options
    nh.setParam("/atrias_gui/main_controller", controllerDataOut.main_controller);
    nh.setParam("/atrias_gui/switch_method", controllerDataOut.switch_method);

    // Gait options
    nh.setParam("/atrias_gui/swing_leg_retraction", controllerDataOut.swing_leg_retraction);
    nh.setParam("/atrias_gui/stance_leg_extension", controllerDataOut.stance_leg_extension);
    nh.setParam("/atrias_gui/leg_length", controllerDataOut.leg_length);
    nh.setParam("/atrias_gui/torso_angle", controllerDataOut.torso_angle);
    nh.setParam("/atrias_gui/q1", controllerDataOut.q1);
    nh.setParam("/atrias_gui/q2", controllerDataOut.q2);
    nh.setParam("/atrias_gui/q3", controllerDataOut.q3);
    nh.setParam("/atrias_gui/q4", controllerDataOut.q4);

    // Leg gains
    nh.setParam("/atrias_gui/leg_pos_kp", controllerDataOut.leg_pos_kp);
    nh.setParam("/atrias_gui/leg_pos_kd", controllerDataOut.leg_pos_kd);
    nh.setParam("/atrias_gui/leg_for_kp", controllerDataOut.leg_for_kp);
    nh.setParam("/atrias_gui/leg_for_ki", controllerDataOut.leg_for_ki);
    nh.setParam("/atrias_gui/leg_for_kd", controllerDataOut.leg_for_kd);

    // Hip gains
    nh.setParam("/atrias_gui/hip_pos_kp", controllerDataOut.hip_pos_kp);
    nh.setParam("/atrias_gui/hip_pos_kd", controllerDataOut.hip_pos_kd);
    nh.setParam("/atrias_gui/left_toe_pos", controllerDataOut.left_toe_pos);
    nh.setParam("/atrias_gui/right_toe_pos", controllerDataOut.right_toe_pos);

    // Torso options
    nh.setParam("/atrias_gui/qvpp", controllerDataOut.qvpp);
    nh.setParam("/atrias_gui/rvpp", controllerDataOut.rvpp);

    // Debug
    nh.setParam("/atrias_gui/current_limit", controllerDataOut.current_limit);
    nh.setParam("/atrias_gui/velocity_limit", controllerDataOut.velocity_limit);
    nh.setParam("/atrias_gui/deflection_limit", controllerDataOut.deflection_limit);
    nh.setParam("/atrias_gui/walking_state", controllerDataOut.walking_state);
}

//! \brief Update the GUI.
void guiUpdate() {
    // Main controller options
    controllerDataOut.main_controller = (uint8_t)main_controller_combobox->get_active_row_number();
    // TODO if not enabled switch back to standing controller.
    controllerDataOut.switch_method = (uint8_t)switch_method_combobox->get_active_row_number();

    // Gait options
    controllerDataOut.swing_leg_retraction = swing_leg_retraction_spinbutton->get_value();
    controllerDataOut.stance_leg_extension = stance_leg_extension_spinbutton->get_value();
    controllerDataOut.leg_length = leg_length_spinbutton->get_value();
    controllerDataOut.torso_angle = torso_angle_spinbutton->get_value();
    controllerDataOut.q1 = q1_spinbutton->get_value();
    controllerDataOut.q2 = q2_spinbutton->get_value();
    controllerDataOut.q3 = q3_spinbutton->get_value();
    controllerDataOut.q4 = q4_spinbutton->get_value();

    // Leg gains
    controllerDataOut.leg_pos_kp = leg_pos_kp_spinbutton->get_value();
    controllerDataOut.leg_pos_kd = leg_pos_kd_spinbutton->get_value();
    controllerDataOut.leg_for_kp = leg_for_kp_spinbutton->get_value();
    controllerDataOut.leg_for_ki = leg_for_ki_spinbutton->get_value();
    controllerDataOut.leg_for_kd = leg_for_kd_spinbutton->get_value();

    // Hip gains
    controllerDataOut.hip_pos_kp = hip_pos_kp_spinbutton->get_value();
    controllerDataOut.hip_pos_kd = hip_pos_kd_spinbutton->get_value();
    controllerDataOut.left_toe_pos = left_toe_pos_spinbutton->get_value();
    controllerDataOut.right_toe_pos = right_toe_pos_spinbutton->get_value();

    // Torso options
    controllerDataOut.qvpp = qvpp_spinbutton->get_value();
    controllerDataOut.rvpp = rvpp_spinbutton->get_value();

    // Debug
    controllerDataOut.current_limit = current_limit_spinbutton->get_value();
    controllerDataOut.velocity_limit = velocity_limit_spinbutton->get_value();
    controllerDataOut.deflection_limit = deflection_limit_spinbutton->get_value();
    walking_state_spinbutton->set_value(controllerDataIn.walking_state);

    // publish the controller input
    pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}
