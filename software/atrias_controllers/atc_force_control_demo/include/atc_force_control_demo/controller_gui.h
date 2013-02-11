/*! \file controller_gui.h
 *  \author Mikhail Jones
 *  \brief Orocos Component code for the atc_force_control_demo controller.
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_force_control_demo/controller_input.h>
#include <atc_force_control_demo/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

// ROS
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data
atc_force_control_demo::controller_input controllerDataOut;
atc_force_control_demo::controller_status controllerDataIn;

// GUI elements
Gtk::SpinButton *leg_pos_p_gain_spinbutton,
	*leg_pos_d_gain_spinbutton,
	*leg_force_p_gain_spinbutton,
	*leg_force_d_gain_spinbutton,
	*fx_spinbutton,
	*fz_spinbutton,
	*ampx_spinbutton,
	*ampz_spinbutton,
	*freqx_spinbutton,
	*freqz_spinbutton,
	*offsetx_spinbutton,
	*offsetz_spinbutton,
	*hip_angle_spinbutton,
	*left_toe_spinbutton,
	*right_toe_spinbutton,
	*hip_p_gain_spinbutton,
	*hip_d_gain_spinbutton,
	*robot_spring_spinbutton,
	*robot_motor_spinbutton,
	*robot_gear_spinbutton;

Gtk::CheckButton *constant_force_checkbutton,
	*sinewave_force_checkbutton,
	*constant_hip_checkbutton,
	*advanced_hip_checkbutton;

// Parameters
//     NOTE: ROS parameters cannot be floats so loss-of-precision issues do not
//     arise when both C and python nodes access the same parameter. See:
//     http://answers.ros.org/question/10938/why-cant-you-use-floats-for-accessing-parameters-in-roscpp/

void controllerCallback(const atc_force_control_demo::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

