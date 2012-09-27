/*
 * controller_gui.cpp
 *
 * atc_demo_range_of_motion controller
 */

#include <atc_demo_range_of_motion/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
    gui->get_widget("position_left_A_spinbutton",      position_left_A_spinbutton);
    gui->get_widget("position_left_B_spinbutton",      position_left_B_spinbutton);
    gui->get_widget("position_left_hip_spinbutton",    position_left_hip_spinbutton);
    gui->get_widget("position_right_A_spinbutton",     position_right_A_spinbutton);
    gui->get_widget("position_right_B_spinbutton",     position_right_B_spinbutton);
    gui->get_widget("position_right_hip_spinbutton",   position_right_hip_spinbutton);
    gui->get_widget("position_leg_motor_p_spinbutton", position_leg_motor_p_spinbutton);
    gui->get_widget("position_leg_motor_d_spinbutton", position_leg_motor_d_spinbutton);
    gui->get_widget("position_hip_motor_p_spinbutton", position_hip_motor_p_spinbutton);
    gui->get_widget("position_hip_motor_d_spinbutton", position_hip_motor_d_spinbutton);
    gui->get_widget("position_leg_duration_spinbutton", position_leg_duration_spinbutton);
    gui->get_widget("position_hip_duration_spinbutton", position_hip_duration_spinbutton);
    gui->get_widget("position_mode_combobox", position_mode_combobox);

    if (position_left_A_spinbutton && position_left_B_spinbutton && position_left_hip_spinbutton &&
        position_right_A_spinbutton && position_right_B_spinbutton && position_right_hip_spinbutton &&
        position_leg_motor_p_spinbutton && position_leg_motor_d_spinbutton &&
        position_hip_motor_p_spinbutton && position_hip_motor_d_spinbutton &&
        position_leg_duration_spinbutton && position_hip_duration_spinbutton &&
        position_mode_combobox) {
        // Set ranges.
        position_left_A_spinbutton->set_range(LEG_A_MOTOR_MIN_LOC        + LEG_LOC_SAFETY_DISTANCE, LEG_A_MOTOR_MAX_LOC     - LEG_LOC_SAFETY_DISTANCE);
        position_left_B_spinbutton->set_range(LEG_B_MOTOR_MIN_LOC        + LEG_LOC_SAFETY_DISTANCE, LEG_B_MOTOR_MAX_LOC     - LEG_LOC_SAFETY_DISTANCE);
        position_left_hip_spinbutton->set_range(LEFT_HIP_MOTOR_MIN_LOC   + HIP_LOC_SAFETY_DISTANCE, LEFT_HIP_MOTOR_MAX_LOC  - HIP_LOC_SAFETY_DISTANCE);
        position_right_A_spinbutton->set_range(LEG_A_MOTOR_MIN_LOC       + LEG_LOC_SAFETY_DISTANCE, LEG_A_MOTOR_MAX_LOC     - LEG_LOC_SAFETY_DISTANCE);
        position_right_B_spinbutton->set_range(LEG_B_MOTOR_MIN_LOC       + LEG_LOC_SAFETY_DISTANCE, LEG_B_MOTOR_MAX_LOC     - LEG_LOC_SAFETY_DISTANCE);
        position_right_hip_spinbutton->set_range(RIGHT_HIP_MOTOR_MIN_LOC + HIP_LOC_SAFETY_DISTANCE, RIGHT_HIP_MOTOR_MAX_LOC - HIP_LOC_SAFETY_DISTANCE);
        position_leg_motor_p_spinbutton->set_range(0, 5000);
        position_leg_motor_d_spinbutton->set_range(0, 300);
        position_hip_motor_p_spinbutton->set_range(0, 100);
        position_hip_motor_d_spinbutton->set_range(0, 10);
        position_leg_duration_spinbutton->set_range(0.55, 5);
        position_hip_duration_spinbutton->set_range(0.1, 5);

        // Set up subscriber and publisher.
        sub = nh.subscribe("atc_demo_range_of_motion_status", 0, controllerCallback);
        pub = nh.advertise<atc_demo_range_of_motion::controller_input>("atc_demo_range_of_motion_input", 0);
        return true;
    }
    return false;
}

//! \brief Update our local copy of the controller status.
void controllerCallback(const atc_demo_range_of_motion::controller_status &status) {
    controllerDataIn = status;
}

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
    // Get parameters in the atrias_gui namespace.
    nh.getParam("/atrias_gui/desLeftAPos", controllerDataOut.desLeftAPos);
    nh.getParam("/atrias_gui/desLeftBPos", controllerDataOut.desLeftBPos);
    nh.getParam("/atrias_gui/desLeftHipPos", controllerDataOut.desLeftHipPos);
    nh.getParam("/atrias_gui/desRightAPos", controllerDataOut.desRightAPos);
    nh.getParam("/atrias_gui/desRightBPos", controllerDataOut.desRightBPos);
    nh.getParam("/atrias_gui/desRightHipPos", controllerDataOut.desRightHipPos);
    nh.getParam("/atrias_gui/leg_p_gain", controllerDataOut.leg_p_gain);
    nh.getParam("/atrias_gui/leg_d_gain", controllerDataOut.leg_d_gain);
    nh.getParam("/atrias_gui/hip_p_gain", controllerDataOut.hip_p_gain);
    nh.getParam("/atrias_gui/hip_d_gain", controllerDataOut.hip_d_gain);
    nh.getParam("/atrias_gui/legDuration", controllerDataOut.legDuration);
    nh.getParam("/atrias_gui/hipDuration", controllerDataOut.hipDuration);

    // Configure the GUI.
    position_left_A_spinbutton->set_value(controllerDataOut.desLeftAPos);
    position_left_B_spinbutton->set_value(controllerDataOut.desLeftBPos);
    position_left_hip_spinbutton->set_value(controllerDataOut.desLeftHipPos);
    position_right_A_spinbutton->set_value(controllerDataOut.desRightAPos);
    position_right_B_spinbutton->set_value(controllerDataOut.desRightBPos);
    position_right_hip_spinbutton->set_value(controllerDataOut.desRightHipPos);
    position_leg_motor_p_spinbutton->set_value(controllerDataOut.leg_p_gain);
    position_leg_motor_d_spinbutton->set_value(controllerDataOut.leg_d_gain);
    position_hip_motor_p_spinbutton->set_value(controllerDataOut.hip_p_gain);
    position_hip_motor_d_spinbutton->set_value(controllerDataOut.hip_d_gain);
    position_leg_duration_spinbutton->set_value(controllerDataOut.legDuration);
    position_hip_duration_spinbutton->set_value(controllerDataOut.hipDuration);
}

//! \brief Set parameters on server according to current GUI settings.
void setParameters() {
    nh.setParam("/atrias_gui/desLeftAPos", controllerDataOut.desLeftAPos);
    nh.setParam("/atrias_gui/desLeftBPos", controllerDataOut.desLeftBPos);
    nh.setParam("/atrias_gui/desLeftHipPos", controllerDataOut.desLeftHipPos);
    nh.setParam("/atrias_gui/desRightAPos", controllerDataOut.desRightAPos);
    nh.setParam("/atrias_gui/desRightBPos", controllerDataOut.desRightBPos);
    nh.setParam("/atrias_gui/desRightHipPos", controllerDataOut.desRightHipPos);
    nh.setParam("/atrias_gui/leg_p_gain", controllerDataOut.leg_p_gain);
    nh.setParam("/atrias_gui/leg_d_gain", controllerDataOut.leg_d_gain);
    nh.setParam("/atrias_gui/hip_p_gain", controllerDataOut.hip_p_gain);
    nh.setParam("/atrias_gui/hip_d_gain", controllerDataOut.hip_d_gain);
    nh.setParam("/atrias_gui/legDuration", controllerDataOut.legDuration);
    nh.setParam("/atrias_gui/hipDuration", controllerDataOut.hipDuration);
}

//! \brief Update the GUI.
void guiUpdate() {
    controllerDataOut.desLeftAPos = position_left_A_spinbutton->get_value();
    controllerDataOut.desLeftBPos = position_left_B_spinbutton->get_value();
    controllerDataOut.desLeftHipPos = position_left_hip_spinbutton->get_value();
    controllerDataOut.desRightAPos = position_right_A_spinbutton->get_value();
    controllerDataOut.desRightBPos = position_right_B_spinbutton->get_value();
    controllerDataOut.desRightHipPos = position_right_hip_spinbutton->get_value();
    controllerDataOut.leg_p_gain = position_leg_motor_p_spinbutton->get_value();
    controllerDataOut.leg_d_gain = position_leg_motor_d_spinbutton->get_value();
    controllerDataOut.hip_p_gain = position_hip_motor_p_spinbutton->get_value();
    controllerDataOut.hip_d_gain = position_hip_motor_d_spinbutton->get_value();
    controllerDataOut.legDuration = position_leg_duration_spinbutton->get_value();
    controllerDataOut.hipDuration = position_hip_duration_spinbutton->get_value();
    controllerDataOut.mode = position_mode_combobox->get_active_row_number();

    pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

