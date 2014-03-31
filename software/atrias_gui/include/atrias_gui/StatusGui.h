/*
 * StatusGui.h
 *
 *  Created on: May 30, 2012
 *      Author: Michael Anderson
 */

#ifndef STATUSGUI_H_
#define STATUSGUI_H_

#include <ros/ros.h>
#include <atrias_shared/drl_math.h>
#include <robot_invariant_defs.h>
#include <robot_variant_defs.h>
#include <gtkmm.h>
#include <atrias_msgs/rt_ops_cycle.h>
#include <atrias_msgs/robot_state_legHalf.h>
#include <atrias_msgs/robot_state_hip.h>

#define CONTROL_LOOP_INTERVAL_USEC 1000.
#define CPU_USAGE_AVERAGE_TICKS 10

using namespace atrias_msgs;

class StatusGui {
public:
    StatusGui(char *path);
    virtual ~StatusGui();

    void update(rt_ops_cycle);

private:
    void update_medulla_errors(uint8_t errorFlags, uint8_t limitSwitches, Gtk::Entry *errorEntry);
    void update_robot_status(rt_ops_cycle);

    // CPU Usage variables
    uint16_t usage[CPU_USAGE_AVERAGE_TICKS];
    uint8_t usageIndex;

    // GUI objects
    Glib::RefPtr<Gtk::Builder> gui;
    Gtk::Window *status_window;

    Gtk::Entry *medullaLAError_entry,
        *medullaLBError_entry,
        *medullaRAError_entry,
        *medullaRBError_entry,
        *medullaLHipError_entry,
        *medullaRHipError_entry,
        *medullaBoomError_entry;

    Gtk::ProgressBar *motor_torqueLeftA_progress_bar,
        *motor_torqueLeftB_progress_bar,
        *motor_torqueLeftHip_progress_bar,
        *motor_torqueRightA_progress_bar,
        *motor_torqueRightB_progress_bar,
        *motor_torqueRightHip_progress_bar;

    Gtk::Entry *azPosDisplay,
        *elPosDisplay,
        *azVelDisplay,
        *elVelDisplay,
        *leftToeDisplay,
        *rightToeDisplay,
        *xPosDisplay,
        *yPosDisplay,
        *zPosDisplay,
        *xVelDisplay,
        *yVelDisplay,
        *zVelDisplay,
        *torqueLeftADisplay,
        *torqueLeftBDisplay,
        *torqueLeftHipDisplay,
        *torqueRightADisplay,
        *torqueRightBDisplay,
        *torqueRightHipDisplay,
        *spring_deflection_left_A_entry,
        *spring_deflection_left_B_entry,
        *spring_deflection_right_A_entry,
        *spring_deflection_right_B_entry,
        *leftLegLengthDisplay,
        *leftLegAngleDisplay,
        *rightLegLengthDisplay,
        *rightLegAngleDisplay,
        *leftHipAngleDisplay,
        *rightHipAngleDisplay,
        *leftALogicVoltage,
        *leftBLogicVoltage,
        *rightALogicVoltage,
        *rightBLogicVoltage,
        *leftAMotorVoltage,
        *leftBMotorVoltage,
        *rightAMotorVoltage,
        *rightBMotorVoltage;
};

#endif /* STATUSGUI_H_ */
