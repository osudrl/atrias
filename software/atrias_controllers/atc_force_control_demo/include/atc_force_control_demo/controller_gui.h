/*! \file controller_gui.h
 *  \author Mikhail Jones
 *  \brief Orocos Component code for the atc_force_control_demo controller.
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_force_control_demo/controller_input.h>
#include <atc_force_control_demo/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_force_control_demo::controller_input controllerDataOut;
atc_force_control_demo::controller_status controllerDataIn;

// GUI elements
Gtk::SpinButton *leg_pos_kp_spinbutton,
    *leg_pos_kd_spinbutton,
    *leg_for_kp_spinbutton,
    *leg_for_kd_spinbutton,
    *hip_pos_kp_spinbutton,
    *hip_pos_kd_spinbutton,
    *robot_ks_spinbutton,
    *robot_kt_spinbutton,
    *robot_kg_spinbutton,
    *left_hip_ang_spinbutton,
    *right_hip_ang_spinbutton,
    *left_toe_pos_spinbutton,
    *right_toe_pos_spinbutton,
    *left_leg_len_spinbutton,
    *right_leg_len_spinbutton,
    *left_leg_ang_spinbutton,
    *right_leg_ang_spinbutton,
    *left_fx_spinbutton,
    *right_fx_spinbutton,
    *left_fz_spinbutton,
    *right_fz_spinbutton,
    *left_ampx_spinbutton,
    *right_ampx_spinbutton,
    *left_ampz_spinbutton,
    *right_ampz_spinbutton,
    *left_freqx_spinbutton,
    *right_freqx_spinbutton,
    *left_freqz_spinbutton,
    *right_freqz_spinbutton,
    *left_offx_spinbutton,
    *right_offx_spinbutton,
    *left_offz_spinbutton,
    *right_offz_spinbutton;

Gtk::RadioButton *constant_hip_radiobutton,
	*constant_toe_radiobutton,
	*left_leg_pos_radiobutton,
	*right_leg_pos_radiobutton,
	*left_leg_for_radiobutton,
	*right_leg_for_radiobutton,
	*left_leg_wave_for_radiobutton,
	*right_leg_wave_for_radiobutton;
	
Gtk::ComboBox *force_combobox;

// Parameters
//     NOTE: ROS parameters cannot be floats so loss-of-precision issues do not
//     arise when both C and python nodes access the same parameter. See:
//     http://answers.ros.org/question/10938/why-cant-you-use-floats-for-accessing-parameters-in-roscpp/

void controllerCallback(const atc_force_control_demo::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

