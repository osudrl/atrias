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
#include <atrias_controllers/data_subscriber_srv.h>

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

Gtk::Label *raibert_state_label;
Gtk::HScale *raibert_desired_velocity_hscale,
            *raibert_desired_height_hscale,
            *raibert_hor_vel_gain_hscale,
            *raibert_leg_force_gain_hscale,
            *raibert_leg_angle_gain_hscale,
            *raibert_stance_p_gain_hscale,
            *raibert_stance_d_gain_hscale,
            *raibert_stance_spring_threshold_hscale,
            *raibert_preferred_leg_len_hscale,
            *raibert_flight_p_gain_hscale,
            *raibert_flight_d_gain_hscale,
            *raibert_flight_spring_threshold_hscale;
Gtk::SpinButton *raibert_desired_velocity_spinbutton,
                *raibert_desired_height_spinbutton,
                *raibert_hor_vel_gain_spinbutton,
                *raibert_leg_force_gain_spinbutton,
                *raibert_leg_angle_gain_spinbutton,
                *raibert_stance_p_gain_spinbutton,
                *raibert_stance_d_gain_spinbutton,
                *raibert_stance_spring_threshold_spinbutton,
                *raibert_preferred_leg_len_spinbutton,
                *raibert_flight_p_gain_spinbutton,
                *raibert_flight_d_gain_spinbutton,
                *raibert_flight_spring_threshold_spinbutton;

Gtk::Image *test_motors_status_image;
Gtk::Image *test_flight_status_image;
Gtk::Label *test_label;
Gtk::HScale *test_slider_flightKP;
Gtk::HScale *test_slider_flightKD;
Gtk::HScale *test_slider_stanceKP;
Gtk::HScale *test_slider_stanceKD;
Gtk::HScale *test_slider_desiredLengthLong;
Gtk::HScale *test_slider_desiredLengthShort;
Gtk::HScale *test_slider_toeSwitchThreshold;
Gtk::HScale *test_slider_springDeflectionThreshold;
Gtk::HScale *test_slider_springDeflectionA;
Gtk::HScale *test_slider_springDeflectionB;

Gtk::HScale *force_control_p_gainA;
Gtk::HScale *force_control_d_gainA;
Gtk::HScale *force_control_i_gainA;
Gtk::HScale *force_control_p_gainB;
Gtk::HScale *force_control_d_gainB;
Gtk::HScale *force_control_i_gainB;
Gtk::HScale *force_control_spring_deflection;

Gtk::DrawingArea *drawing_area;

Gtk::ProgressBar *motor_torqueA_progress_bar;
Gtk::ProgressBar *motor_torqueB_progress_bar;

Gtk::ProgressBar *motor_velocityA_progress_bar;
Gtk::ProgressBar *motor_velocityB_progress_bar;

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

Gtk::Entry *velocityADisplay;
Gtk::Entry *velocityBDisplay;

Gtk::Entry *spring_deflection_A_entry;
Gtk::Entry *spring_deflection_B_entry;

Gtk::Button *restart_button;
Gtk::Button *enable_button;
Gtk::Button *disable_button;

Cairo::RefPtr<Cairo::Context> cr;
Gtk::Allocation drawing_allocation;

//FILE *logFile;

ros::ServiceClient atrias_client;
ros::ServiceClient datalog_client;
atrias_controllers::atrias_srv atrias_srv;
atrias_controllers::data_subscriber_srv data_subscriber_srv;

double last_p_gain;
double last_d_gain;

long nextLogTime;

bool poke_controller( void );
std::string format_float ( float );

void log_chkbox_toggled( void );

void restart_robot( void );
void enable_motors( void );
void disable_motors( void );

void switch_controllers(GtkNotebookPage *, guint);

void draw_leg();
