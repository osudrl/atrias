// Devin Koepl

#include <stdlib.h>
#include <gtkmm.h>
#include <stdlib.h>
#include <cairomm/context.h>

#include <ros/ros.h>

#include <atrias/ucontroller.h>

#include <atrias_controllers/controller.h>
#include <atrias_controllers/atrias_srv.h>

#include <drl_library/drl_math.h>

// GUI objects
Gtk::Window *window;

Gtk::Notebook *controller_notebook;

Gtk::HScale *motor_torqueA_hscale;
Gtk::HScale *motor_torqueB_hscale;

Gtk::HScale *motor_positionA_hscale;
Gtk::HScale *motor_positionB_hscale;
Gtk::HScale *p_motor_position_hscale;
Gtk::HScale *d_motor_position_hscale;

Gtk::HScale *leg_length_torque_hscale;
Gtk::HScale *leg_angle_torque_hscale;

Gtk::HScale *leg_length_hscale;
Gtk::HScale *leg_angle_hscale;
Gtk::HScale *p_leg_position_hscale;
Gtk::HScale *d_leg_position_hscale;

Gtk::HScale *leg_angle_amplitude_hscale;
Gtk::HScale *leg_angle_frequency_hscale;
Gtk::HScale *leg_length_amplitude_hscale;
Gtk::HScale *leg_length_frequency_hscale;
Gtk::HScale *p_sine_wave_hscale;
Gtk::HScale *d_sine_wave_hscale;

Gtk::HScale *raibert_desired_velocity_hscale;
Gtk::HScale *raibert_desired_height_hscale;
Gtk::HScale *raibert_hor_vel_gain_hscale;
Gtk::HScale *raibert_leg_force_gain_hscale;
Gtk::HScale *raibert_leg_angle_gain_hscale;
Gtk::HScale *raibert_stance_p_gain_hscale;
Gtk::HScale *raibert_stance_d_gain_hscale;
Gtk::HScale *raibert_stance_spring_threshold_hscale;
Gtk::Label  *raibert_state_label;
Gtk::HScale *raibert_preferred_leg_len_hscale;
Gtk::HScale *raibert_flight_p_gain_hscale;
Gtk::HScale *raibert_flight_d_gain_hscale;
Gtk::HScale *raibert_flight_spring_threshold_hscale;

//NEW
Gtk::SpinButton *extra_1_line_1_object;
Gtk::SpinButton *extra_1_line_2_object;
Gtk::HScale *extra_1_line_3_object;
Gtk::HScale *extra_1_line_4_object;
Gtk::RadioButton *extra_1_line_5_object_1;
Gtk::RadioButton *extra_1_line_5_object_2;
Gtk::RadioButton *extra_1_line_5_object_3;
Gtk::RadioButton *extra_1_line_5_object_4;
//END NEW

Gtk::DrawingArea *drawing_area;

Gtk::ProgressBar *motor_torqueA_progress_bar;
Gtk::ProgressBar *motor_torqueB_progress_bar;

Gtk::CheckButton *log_file_chkbox;
Gtk::FileChooser *log_file_chooser;

Gtk::Label * hor_vel_label;
Gtk::Label * height_label;

Gtk::Button * restart_button;
Gtk::Button * enable_button;
Gtk::Button * disable_button;

Cairo::RefPtr<Cairo::Context> cr;
Gtk::Allocation drawing_allocation;

//FILE *logFile;

ros::ServiceClient atrias_client;
atrias_controllers::atrias_srv atrias_srv;

FILE *log_file_fp;

bool poke_controller( void );

void log_chkbox_toggled( void );

void restart_robot( void );
void enable_motors( void );
void disable_motors( void );

void switch_controllers(GtkNotebookPage *, guint);

void draw_leg();
