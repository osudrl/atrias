/*
 * controller_gui.h
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_leg_sin_wave/controller_input.h>
#include <atc_leg_sin_wave/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <robot_invariant_defs.h>
#include <ros/ros.h>

ros::NodeHandle nh;

ros::Publisher pub;
ros::Subscriber sub;

atc_leg_sin_wave::controller_input controllerDataOut;
atc_leg_sin_wave::controller_status controllerDataIn;

Gtk::HScale *leg_angle_amplitude_hscale,
            *leg_angle_frequency_hscale,
            *leg_length_amplitude_hscale,
            *leg_length_frequency_hscale,
	    *hip_angle_amplitude_hscale,
	    *hip_angle_frequency_hscale;

Gtk::SpinButton *leg_p_gain_entry,
	        *leg_d_gain_entry,
	        *hip_p_gain_entry,
	        *hip_d_gain_entry;

// Parameters
//     NOTE: ROS parameters cannot be floats so loss-of-precision issues do not
//     arise when both C and python nodes access the same parameter. See:
//     http://answers.ros.org/question/10938/why-cant-you-use-floats-for-accessing-parameters-in-roscpp/
double leg_angle_amplitude_param;
double leg_angle_frequency_param;
double leg_length_amplitude_param;
double leg_length_frequency_param;
double p_gain_param;
double d_gain_param;

void controllerCallback(const atc_leg_sin_wave::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */
