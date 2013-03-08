/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_force_control_hopping/controller_input.h>
#include <atc_force_control_hopping/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS -------------------------------------------------------------------------
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data ------------------------------------------------------------------------
atc_force_control_hopping::controller_input controllerDataOut;
atc_force_control_hopping::controller_status controllerDataIn;

// GUI elements ----------------------------------------------------------------
Gtk::RadioButton *appex_radiobutton,
	*terrain_radiobutton,
	*two_leg_radiobutton,
	*alt_leg_radiobutton,
	*left_leg_radiobutton,
	*right_leg_radiobutton,
	*stand_radiobutton,
	*hop_radiobutton;

Gtk::CheckButton *hip_checkbutton,
	*deinit_checkbutton;

Gtk::SpinButton *left_toe_spinbutton,
	*right_toe_spinbutton,
	*slip_h_spinbutton,
	*stand_r0_spinbutton,
	*slip_r0_spinbutton,
	*slip_m_spinbutton,
	*leg_pos_kp_spinbutton,
	*leg_for_kp_spinbutton,
	*leg_pos_kd_spinbutton,
	*leg_for_kd_spinbutton,
	*hip_kp_spinbutton,
	*hip_kd_spinbutton,
	*robot_ks_spinbutton,
	*robot_kt_spinbutton,
	*robot_kg_spinbutton;

// Parameters ------------------------------------------------------------------
//     NOTE: ROS parameters cannot be floats so loss-of-precision issues do not
//     arise when both C and python nodes access the same parameter. See:
//     http://answers.ros.org/question/10938/why-cant-you-use-floats-for-accessing-parameters-in-roscpp/

void controllerCallback(const atc_force_control_hopping::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

