/*
 * StatusGui.h
 *
 *  Created on: May 30, 2012
 *      Author: Michael Anderson
 */

#ifndef STATUSGUI_H_
#define STATUSGUI_H_

#include <ros/ros.h>
#include <drl_library/drl_math.h>
#include <atrias_control/ucontroller.h>
#include <gtkmm.h>
#include <atrias_msgs/controller_status.h>

#define CONTROL_LOOP_INTERVAL_USEC 1000.
#define CPU_USAGE_AVERAGE_TICKS 10

using namespace atrias_msgs;

class StatusGui {
public:
	StatusGui(char *path);
	virtual ~StatusGui();

	void update(controller_status);

private:
	void update_medulla_status(controller_status);
	void decode_limit_switch(Glib::ustring *error, uint8_t switches);
	void update_robot_status(controller_status);

	//CPU Usage variables
	uint16_t usage[CPU_USAGE_AVERAGE_TICKS];
	uint8_t usageIndex;

	// GUI objects
	Glib::RefPtr<Gtk::Builder> gui;
	Gtk::Window *status_window;

	/*
	 * Medulla Status
	 */
	Gtk::Entry  *MedullaA_TempA,
	            *MedullaA_TempB,
	            *MedullaA_TempC,
	            *MedullaA_VLogic,
	            *MedullaA_VMotor;

	Gtk::Entry  *MedullaB_TempA,
	            *MedullaB_TempB,
	            *MedullaB_TempC,
	            *MedullaB_VLogic,
	            *MedullaB_VMotor;

	Gtk::Entry  *medullaAError_entry,
		    *medullaBError_entry,
		    *medullaHipError_entry,
		    *medullaBoomError_entry;
	/*
	 * End Medulla Status
	 */

	Gtk::Label *cpu_load_label;

	Gtk::ProgressBar *motor_torqueA_progress_bar,
	                 *motor_torqueB_progress_bar,
	                 *motor_torqueHip_progress_bar,
	                 *motor_velocityA_progress_bar,
	                 *motor_velocityB_progress_bar,
	                 *motor_velocityHip_progress_bar,
	                 *spring_deflectionA_progress_bar,
	                 *spring_deflectionB_progress_bar,
	                 *cpu_load_bar;

	Gtk::Entry  *xPosDisplay,
	            *yPosDisplay,
	            *zPosDisplay,
	            *xVelDisplay,
	            *yVelDisplay,
	            *zVelDisplay,
	            *torqueADisplay,
	            *torqueBDisplay,
	            *torqueHipDisplay,
	            *velocityADisplay,
	            *velocityBDisplay,
	            *velocityHipDisplay,
	            *spring_deflection_A_entry,
	            *spring_deflection_B_entry,
	            *legLengthDisplay,
	            *legAngleDisplay;
};

#endif /* STATUSGUI_H_ */
