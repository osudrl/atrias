/*! \file controller_component.h
 *  \author Mikhail Jones
 *  \brief Orocos Component header for atc_vertical_force_control_hopping controller.
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_vertical_force_control_hopping/controller_input.h>
#include <atc_vertical_force_control_hopping/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_vertical_force_control_hopping::controller_input controllerDataOut;
atc_vertical_force_control_hopping::controller_status controllerDataIn;

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
	*left_toe_pos_spinbutton,
	*right_toe_pos_spinbutton,
	*left_leg_len_spinbutton,
	*right_leg_len_spinbutton,
	*left_leg_ang_spinbutton,
	*right_leg_ang_spinbutton,
	*slip_h_spinbutton,
	*slip_l0_spinbutton,
	*slip_m_spinbutton,
	*slip_k_spinbutton;

Gtk::CheckButton *hip_contr_checkbutton,
	*debug1_checkbutton;

Gtk::RadioButton *leg_pos_contr_radiobutton,
	*leg_slip_contr_radiobutton,
	*left_leg_hop_radiobutton,
	*right_leg_hop_radiobutton,
	*two_leg_hop_radiobutton,
	*alt_leg_hop_radiobutton;

// Parameters
//     NOTE: ROS parameters cannot be floats so loss-of-precision issues do not
//     arise when both C and python nodes access the same parameter. See:
//     http://answers.ros.org/question/10938/why-cant-you-use-floats-for-accessing-parameters-in-roscpp/

void controllerCallback(const atc_vertical_force_control_hopping::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

