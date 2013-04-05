/*! \file controller_gui.cpp
 *  \author Mikhail Jones
 */

#ifndef CONTROLLER_GUI_H_
#define CONTROLLER_GUI_H_

#include <atc_slip_hopping/controller_input.h>
#include <atc_slip_hopping/controller_status.h>
#include <atrias_shared/gui_library.h>
#include <ros/ros.h>

// ROS -------------------------------------------------------------------------
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Publisher pub;

// Data ------------------------------------------------------------------------
atc_slip_hopping::controller_input controllerDataOut;
atc_slip_hopping::controller_status controllerDataIn;

// GUI elements ----------------------------------------------------------------
//Gtk::SpinButton *example_spinbutton;

// Parameters
//     NOTE: ROS parameters cannot be floats so loss-of-precision issues do not
//     arise when both C and python nodes access the same parameter. See:
//     http://answers.ros.org/question/10938/why-cant-you-use-floats-for-accessing-parameters-in-roscpp/

void controllerCallback(const atc_slip_hopping::controller_status &status);

#endif /* CONTROLLER_GUI_H_ */

