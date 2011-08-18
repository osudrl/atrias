// Devin Koepl

#include <stdlib.h>
#include <stdarg.h>
#include <gtkmm.h>
#include <cairomm/context.h>

#include <time.h>
#include <string.h>

#include <ros/ros.h>

#include <atrias/ucontroller.h>

#include <atrias_controllers/controller.h>
#include <atrias_controllers/atrias_srv.h>

#include <drl_library/drl_math.h>

// GUI objects
Gtk::Window *window;

Gtk::Notebook *controller_notebook;

std::string red_image_path;
std::string green_image_path;

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
Gtk::SpinButton *p_leg_position_spin;
Gtk::SpinButton *d_leg_position_spin;

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
Gtk::Label *raibert_state_label;
Gtk::HScale *raibert_preferred_leg_len_hscale;
Gtk::HScale *raibert_flight_p_gain_hscale;
Gtk::HScale *raibert_flight_d_gain_hscale;
Gtk::HScale *raibert_flight_spring_threshold_hscale;

Gtk::Image *test_motors_status_image;
Gtk::Image *test_flight_status_image;
Gtk::Label *test_label;
Gtk::HScale *test_slider_flightGainP;
Gtk::HScale *test_slider_flightGainD;
Gtk::HScale *test_slider_stanceGainP;
Gtk::HScale *test_slider_stanceGainD;
Gtk::HScale *test_slider_desiredLength;
Gtk::HScale *test_slider_activationDeflection;

/*
Gtk::HScale *grizzle_stance_threshold_hscale;
Gtk::HScale *grizzle_flight_threshold_hscale;
Gtk::HScale *grizzle_motor_gain_hscale;
*/

Gtk::DrawingArea *drawing_area;

Gtk::ProgressBar *motor_torqueA_progress_bar;
Gtk::ProgressBar *motor_torqueB_progress_bar;

Gtk::CheckButton *log_file_chkbox;
Gtk::SpinButton *log_frequency_spin;
Gtk::FileChooserButton *log_file_chooser;

Gtk::Entry *xPosDisplay;
Gtk::Entry *yPosDisplay;
Gtk::Entry *zPosDisplay;

Gtk::Entry *xVelDisplay;
Gtk::Entry *yVelDisplay;
Gtk::Entry *zVelDisplay;

Gtk::Entry *torqueADisplay;
Gtk::Entry *torqueBDisplay;

Gtk::Button *restart_button;
Gtk::Button *enable_button;
Gtk::Button *disable_button;

Cairo::RefPtr<Cairo::Context> cr;
Gtk::Allocation drawing_allocation;

//FILE *logFile;

ros::ServiceClient atrias_client;
atrias_controllers::atrias_srv atrias_srv;

FILE *log_file_fp;

double last_p_gain;
double last_d_gain;

bool isLogging = false;
long nextLogTime;

bool poke_controller( void );
std::string format_float ( float );

void log_chkbox_toggled( void );

void restart_robot( void );
void enable_motors( void );
void disable_motors( void );

void switch_controllers(GtkNotebookPage *, guint);

void draw_leg();
