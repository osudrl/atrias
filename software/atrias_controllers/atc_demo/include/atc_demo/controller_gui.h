/**
 * @file controller_gui.hpp
 * @author Mikhail Jones
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_demo/controller_input.h>
#include <atc_demo/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_demo::controller_input controllerDataOut;
atc_demo::controller_status controllerDataIn;

// GUI elements
Gtk::ComboBox *main_controller_combobox;
    
Gtk::SpinButton *leg_pos_kp_spinbutton,
    *leg_for_kp_spinbutton,
    *leg_for_ki_spinbutton,
    *leg_pos_kd_spinbutton,
    *leg_for_kd_spinbutton,
    *hip_pos_kp_spinbutton,
    *left_toe_pos_spinbutton,
    *hip_pos_kd_spinbutton;

void controllerCallback(const atc_demo::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

