/*
 * controller_gui.cpp
 *
 * atc_joint_position controller
 *
 *  Created on: May 5, 2012
 *      Author: Michael Anderson
 */

#include <atc_joint_position/controller_gui.h>

//! \brief Initialize the GUI.
bool guiInit(Glib::RefPtr<Gtk::Builder> gui) {
	gui->get_widget("p_hl_spinbutton", p_hl_spinbutton);
	gui->get_widget("d_hl_spinbutton", d_hl_spinbutton);
	gui->get_widget("a_hl_spinbutton", a_hl_spinbutton);
	gui->get_widget("p_hr_spinbutton", p_hr_spinbutton);
	gui->get_widget("d_hr_spinbutton", d_hr_spinbutton);
	gui->get_widget("a_hr_spinbutton", a_hr_spinbutton);
	gui->get_widget("p_ll_spinbutton", p_ll_spinbutton);
	gui->get_widget("d_ll_spinbutton", d_ll_spinbutton);
	gui->get_widget("a_ll_spinbutton", a_ll_spinbutton);
	gui->get_widget("l_ll_spinbutton", l_ll_spinbutton);
	gui->get_widget("p_lr_spinbutton", p_lr_spinbutton);
	gui->get_widget("d_lr_spinbutton", d_lr_spinbutton);
	gui->get_widget("a_lr_spinbutton", a_lr_spinbutton);
	gui->get_widget("l_lr_spinbutton", l_lr_spinbutton);
	gui->get_widget("maxLegSpd",       max_leg_spd_spinbutton);
	gui->get_widget("maxHipSpd",       max_hip_spd_spinbutton);
	gui->get_widget("hip_inv_kine_chkbox", vertical_checkbutton);
	gui->get_widget("sync_checkbutton",    sync_checkbutton);
	gui->get_widget("smoothLegMotionChkBtn",  smooth_leg_motion_checkbutton);
	gui->get_widget("smoothHipMotionChkBtn",  smooth_hip_motion_checkbutton);
	
	if (p_hl_spinbutton && d_hl_spinbutton && a_hl_spinbutton &&
	    p_hr_spinbutton && d_hr_spinbutton && a_hr_spinbutton &&
	    p_ll_spinbutton && d_ll_spinbutton && a_ll_spinbutton && l_ll_spinbutton &&
	    p_lr_spinbutton && d_lr_spinbutton && a_lr_spinbutton && l_lr_spinbutton &&
	    max_leg_spd_spinbutton && max_hip_spd_spinbutton && vertical_checkbutton &&
	    sync_checkbutton && smooth_leg_motion_checkbutton && smooth_hip_motion_checkbutton) {
		// Set ranges.
		p_hl_spinbutton->set_range(  0,  500);
		d_hl_spinbutton->set_range(  0,   20);
		a_hl_spinbutton->set_range(-10,   10);
		p_hr_spinbutton->set_range(  0,  500);
		d_hr_spinbutton->set_range(  0,   20);
		a_hr_spinbutton->set_range(-10,   10);
		p_ll_spinbutton->set_range(  0, 6000);
		d_ll_spinbutton->set_range(  0,  200);
		a_ll_spinbutton->set_range(  0,  6.3);
		l_ll_spinbutton->set_range(0.2,    1);
		p_lr_spinbutton->set_range(  0, 6000);
		d_lr_spinbutton->set_range(  0,  200);
		a_lr_spinbutton->set_range(  0,  6.3);
		l_lr_spinbutton->set_range(0.2,    1);
		max_leg_spd_spinbutton->set_range(0.0, 2.0);
		max_hip_spd_spinbutton->set_range(0.0, 0.3);

		// Set values
		p_hl_spinbutton->set_value(150);
		d_hl_spinbutton->set_value(10);
		a_hl_spinbutton->set_value(1.5 * M_PI);
		p_hr_spinbutton->set_value(150);
		d_hr_spinbutton->set_value(10);
		a_hr_spinbutton->set_value(1.5 * M_PI);
		p_ll_spinbutton->set_value(1000);
		d_ll_spinbutton->set_value(10);
		a_ll_spinbutton->set_value(M_PI / 2);
		l_ll_spinbutton->set_value(.8);
		p_lr_spinbutton->set_value(1000);
		d_lr_spinbutton->set_value(10);
		a_lr_spinbutton->set_value(M_PI / 2);
		l_lr_spinbutton->set_value(.8);
		max_leg_spd_spinbutton->set_value(.3);
		max_hip_spd_spinbutton->set_value(.03);

		// Set all the checkboxes active by default.
		vertical_checkbutton->set_active();
		sync_checkbutton->set_active();
		smooth_leg_motion_checkbutton->set_active();
		smooth_hip_motion_checkbutton->set_active();

		// Set up subscriber and publisher.
		pub = nh.advertise<atc_joint_position::controller_input>("ATCJointPosition_input", 0);
		return true;
	}
	return false;
}

//! \brief Update our local copy of the controller status.
/*void controllerCallback(const atc_joint_position::controller_status &status) {
  controllerDataIn = status;
  }*/

//! \brief Get parameters from the server and configure GUI accordingly.
void getParameters() {
	// Get parameters in the atrias_gui namespace.
	nh.getParam("/atrias_gui/p_hl", controllerDataOut.p_hl);
	nh.getParam("/atrias_gui/d_hl", controllerDataOut.d_hl);
	nh.getParam("/atrias_gui/a_hl", controllerDataOut.a_hl);
	nh.getParam("/atrias_gui/p_hr", controllerDataOut.p_hr);
	nh.getParam("/atrias_gui/d_hr", controllerDataOut.d_hr);
	nh.getParam("/atrias_gui/a_hr", controllerDataOut.a_hr);
	nh.getParam("/atrias_gui/p_ll", controllerDataOut.p_ll);
	nh.getParam("/atrias_gui/d_ll", controllerDataOut.d_ll);
	nh.getParam("/atrias_gui/a_ll", controllerDataOut.a_ll);
	nh.getParam("/atrias_gui/l_ll", controllerDataOut.l_ll);
	nh.getParam("/atrias_gui/p_lr", controllerDataOut.p_lr);
	nh.getParam("/atrias_gui/d_lr", controllerDataOut.d_lr);
	nh.getParam("/atrias_gui/a_lr", controllerDataOut.a_lr);
	nh.getParam("/atrias_gui/l_lr", controllerDataOut.l_lr);
	nh.getParam("/atrias_gui/max_leg_spd", controllerDataOut.maxLegSpd);
	nh.getParam("/atrias_gui/max_hip_spd", controllerDataOut.maxHipSpd);
	//nh.getParam("/atrias_gui/vertical", controllerDataOut.vertical);
	//nh.getParam("/atrias_gui/sync", controllerDataOut.sync);

	// Configure the GUI.
	p_hl_spinbutton->set_value(controllerDataOut.p_hl);
	d_hl_spinbutton->set_value(controllerDataOut.d_hl);
	a_hl_spinbutton->set_value(controllerDataOut.a_hl);
	p_hr_spinbutton->set_value(controllerDataOut.p_hr);
	d_hr_spinbutton->set_value(controllerDataOut.d_hr);
	a_hr_spinbutton->set_value(controllerDataOut.a_hr);
	p_ll_spinbutton->set_value(controllerDataOut.p_ll);
	d_ll_spinbutton->set_value(controllerDataOut.d_ll);
	a_ll_spinbutton->set_value(controllerDataOut.a_ll);
	l_ll_spinbutton->set_value(controllerDataOut.l_ll);
	p_lr_spinbutton->set_value(controllerDataOut.p_lr);
	d_lr_spinbutton->set_value(controllerDataOut.d_lr);
	a_lr_spinbutton->set_value(controllerDataOut.a_lr);
	l_lr_spinbutton->set_value(controllerDataOut.l_lr);
	max_leg_spd_spinbutton->set_value(controllerDataOut.maxLegSpd);
	max_hip_spd_spinbutton->set_value(controllerDataOut.maxHipSpd);
}

//! \brief Set spinbutton->get_value()eters on server according to current GUI settings.
void setParameters() {
	nh.setParam("/atrias_gui/p_hl", controllerDataOut.p_hl);
	nh.setParam("/atrias_gui/d_hl", controllerDataOut.d_hl);
	nh.setParam("/atrias_gui/a_hl", controllerDataOut.a_hl);
	nh.setParam("/atrias_gui/p_hr", controllerDataOut.p_hr);
	nh.setParam("/atrias_gui/d_hr", controllerDataOut.d_hr);
	nh.setParam("/atrias_gui/a_hr", controllerDataOut.a_hr);
	nh.setParam("/atrias_gui/p_ll", controllerDataOut.p_ll);
	nh.setParam("/atrias_gui/d_ll", controllerDataOut.d_ll);
	nh.setParam("/atrias_gui/a_ll", controllerDataOut.a_ll);
	nh.setParam("/atrias_gui/l_ll", controllerDataOut.l_ll);
	nh.setParam("/atrias_gui/p_lr", controllerDataOut.p_lr);
	nh.setParam("/atrias_gui/d_lr", controllerDataOut.d_lr);
	nh.setParam("/atrias_gui/a_lr", controllerDataOut.a_lr);
	nh.setParam("/atrias_gui/l_lr", controllerDataOut.l_lr);
	nh.setParam("/atrias_gui/max_leg_spd", controllerDataOut.maxLegSpd);
	nh.setParam("/atrias_gui/max_hip_spd", controllerDataOut.maxHipSpd);
	//nh.setParam("/atrias_gui/vertical", controllerDataOut.vertical);
	//nh.setParam("/atrias_gui/sync", controllerDataOut.sync);
}

//! \brief Update the GUI.
void guiUpdate() {
	if (sync_checkbutton->get_active()) {
		p_lr_spinbutton->set_value(p_ll_spinbutton->get_value());
		d_lr_spinbutton->set_value(d_ll_spinbutton->get_value());
		a_lr_spinbutton->set_value(a_ll_spinbutton->get_value());
		l_lr_spinbutton->set_value(l_ll_spinbutton->get_value());
	}
	controllerDataOut.p_hl      = p_hl_spinbutton->get_value();
	controllerDataOut.d_hl      = d_hl_spinbutton->get_value();
	controllerDataOut.a_hl      = a_hl_spinbutton->get_value();
	controllerDataOut.p_hr      = p_hr_spinbutton->get_value();
	controllerDataOut.d_hr      = d_hr_spinbutton->get_value();
	controllerDataOut.a_hr      = a_hr_spinbutton->get_value();
	controllerDataOut.p_ll      = p_ll_spinbutton->get_value();
	controllerDataOut.d_ll      = d_ll_spinbutton->get_value();
	controllerDataOut.a_ll      = a_ll_spinbutton->get_value();
	controllerDataOut.l_ll      = l_ll_spinbutton->get_value();
	controllerDataOut.p_lr      = p_lr_spinbutton->get_value();
	controllerDataOut.d_lr      = d_lr_spinbutton->get_value();
	controllerDataOut.a_lr      = a_lr_spinbutton->get_value();
	controllerDataOut.l_lr      = l_lr_spinbutton->get_value();
	controllerDataOut.maxLegSpd = max_leg_spd_spinbutton->get_value();
	controllerDataOut.maxHipSpd = max_hip_spd_spinbutton->get_value();
	controllerDataOut.vertical        = vertical_checkbutton->get_active();
	controllerDataOut.smoothLegMotion = smooth_leg_motion_checkbutton->get_active();
	controllerDataOut.smoothHipMotion = smooth_hip_motion_checkbutton->get_active();
	pub.publish(controllerDataOut);
}

//! \brief Take down the GUI.
void guiTakedown() {
}

// vim: noexpandtab
