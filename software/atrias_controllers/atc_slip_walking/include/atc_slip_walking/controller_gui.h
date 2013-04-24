#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_slip_walking/controller_input.h>
#include <atc_slip_walking/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_slip_walking::controller_input controllerDataOut;
atc_slip_walking::controller_status controllerDataIn;

// GUI elements
Gtk::SpinButton *standing_leg_length_spinbutton,
    *touchdown_angle_spinbutton,
    *takeoff_angle_spinbutton,
    *slip_leg_length_spinbutton,
    *linear_spring_constant_spinbutton,
    *leg_stance_kp_spinbutton,
    *leg_flight_kp_spinbutton,
    *leg_stance_kd_spinbutton,
    *leg_flight_kd_spinbutton,
    *hip_kp_spinbutton,
    *hip_kd_spinbutton;

Gtk::ComboBox *main_controller_combobox,
    *spring_type_combobox,
    *stance_controller_combobox;

void controllerCallback(const atc_slip_walking::controller_status &status);

#endif
