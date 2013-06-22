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
    *gait_transitions_combobox;
    
Gtk::SpinButton *atrias_spring_spinbutton,
    *slip_leg_spinbutton,
    *stance_leg_target_spinbutton,
    *flight_leg_target_spinbutton,
    *swing_leg_retraction_spinbutton,
    *pushoff_force_spinbutton,
    *force_threshold_td_spinbutton,
    *force_threshold_to_spinbutton,
    *position_threshold_td_spinbutton,
    *leg_pos_kp_spinbutton,
    *leg_for_kp_spinbutton,
    *leg_for_ki_spinbutton,
    *leg_pos_kd_spinbutton,
    *leg_for_kd_spinbutton,
    *hip_pos_kp_spinbutton,
    *left_toe_pos_spinbutton,
    *hip_pos_kd_spinbutton,
    *right_toe_pos_spinbutton,
    *walking_state_spinbutton,
    *td_position_spinbutton,
    *td_force_spinbutton,
    *to_force_spinbutton;

Gtk::Button *flight_to_button,
    *flight_td_button;

void controllerCallback(const atc_slip_walking::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */
