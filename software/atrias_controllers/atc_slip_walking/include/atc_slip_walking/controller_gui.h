/**
 * @file controller_gui.hpp
 * @author Mikhail Jones
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_slip_walking/controller_input.h>
#include <atc_slip_walking/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_slip_walking::controller_input controllerDataOut;
atc_slip_walking::controller_status controllerDataIn;

// GUI elements
Gtk::ComboBox *main_controller_combobox,
    *switch_method_combobox;
    
Gtk::SpinButton *swing_leg_retraction_spinbutton,
    *stance_leg_extension_spinbutton,
    *leg_length_spinbutton,
    *torso_angle_spinbutton,
    *q1_spinbutton,
    *q2_spinbutton,
    *q3_spinbutton,
    *q4_spinbutton,
    *leg_pos_kp_spinbutton,
    *leg_pos_kd_spinbutton,
    *leg_for_kp_spinbutton,
    *leg_for_ki_spinbutton,
    *leg_for_kd_spinbutton,
    *hip_pos_kp_spinbutton,
    *hip_pos_kd_spinbutton,
    *left_toe_pos_spinbutton,
    *right_toe_pos_spinbutton,
    *current_limit_spinbutton,
    *velocity_limit_spinbutton,
    *deflection_limit_spinbutton,
    *walking_state_spinbutton;

Gtk::Button *flight_to_button,
    *flight_td_button;

void controllerCallback(const atc_slip_walking::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */
