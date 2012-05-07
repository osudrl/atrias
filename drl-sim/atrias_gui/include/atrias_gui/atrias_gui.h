// Devin Koepl

#include <stdlib.h>
#include <stdarg.h>
#include <gtkmm.h>
#include <cairomm/context.h>

#include <time.h>
#include <string.h> /* where is this used? */

#include <ros/ros.h>

#include <atrias/ucontroller.h>

#include <atrias_controllers/controller.h>
#include <atrias_controllers/data_subscriber_srv.h>

#include <atrias_msgs/atrias_controller_requests.h>
#include <atrias_msgs/atrias_data.h>

#include <drl_library/drl_math.h>

// GUI objects
Gtk::Window *window;

Gtk::Notebook *controller_notebook;

std::string red_image_path, 
            green_image_path;

Gtk::HScale *motor_torqueA_hscale,
            *motor_torqueB_hscale,
	    *motor_torque_hip_hscale;

Gtk::HScale *motor_positionA_hscale,
            *motor_positionB_hscale,
            *p_motor_position_hscale,
            *d_motor_position_hscale;

Gtk::HScale *leg_length_torque_hscale,
            *leg_angle_torque_hscale,
            *leg_length_hscale,
            *leg_angle_hscale,
            *p_leg_position_hscale,
            *d_leg_position_hscale,
	    *hip_position_ang,
	    *hip_position_p,
	    *hip_position_d;

Gtk::SpinButton *p_leg_position_spin, 
                *d_leg_position_spin;

Gtk::HScale *leg_angle_amplitude_hscale,
            *leg_angle_frequency_hscale,
            *leg_length_amplitude_hscale,
            *leg_length_frequency_hscale,
            *p_sine_wave_hscale,
            *d_sine_wave_hscale;

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
            *raibert_flight_spring_threshold_hscale,
            *raibert_stance_hip_p_gain,
            *raibert_stance_hip_d_gain,
            *raibert_flight_hip_p_gain,
            *raibert_flight_hip_d_gain;

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

Gtk::Label *hubicki_state_label;
Gtk::HScale *hubicki_desired_velocity_hscale,
            *hubicki_desired_height_hscale,
            *hubicki_hor_vel_gain_hscale,
            *hubicki_leg_force_gain_hscale,
            *hubicki_leg_angle_gain_hscale,
            *hubicki_stance_p_gain_hscale,
            *hubicki_stance_d_gain_hscale,
            *hubicki_stance_spring_threshold_hscale,
            *hubicki_preferred_leg_len_hscale,
            *hubicki_flight_p_gain_hscale,
            *hubicki_flight_d_gain_hscale,
            *hubicki_flight_spring_threshold_hscale,
            *hubicki_stance_hip_p_gain,
            *hubicki_stance_hip_d_gain,
            *hubicki_flight_hip_p_gain,
            *hubicki_flight_hip_d_gain;

Gtk::SpinButton *hubicki_desired_velocity_spinbutton,
                *hubicki_desired_height_spinbutton,
                *hubicki_hor_vel_gain_spinbutton,
                *hubicki_leg_force_gain_spinbutton,
                *hubicki_leg_angle_gain_spinbutton,
                *hubicki_stance_p_gain_spinbutton,
                *hubicki_stance_d_gain_spinbutton,
                *hubicki_stance_spring_threshold_spinbutton,
                *hubicki_preferred_leg_len_spinbutton,
                *hubicki_flight_p_gain_spinbutton,
                *hubicki_flight_d_gain_spinbutton,
                *hubicki_flight_spring_threshold_spinbutton;

Gtk::Image  *test_motors_status_image,
            *test_flight_status_image;

Gtk::Label *test_label;
Gtk::HScale *test_slider_flightKP,
            *test_slider_flightKD,
            *test_slider_stanceKP,
            *test_slider_stanceKD,
            *test_slider_desiredLengthLong,
            *test_slider_desiredLengthShort,
            *test_slider_toeSwitchThreshold,
            *test_slider_springDeflectionThreshold,
            *test_slider_springDeflectionA,
            *test_slider_springDeflectionB;

Gtk::HScale *force_control_p_gainA,
            *force_control_d_gainA,
            *force_control_i_gainA,
            *force_control_p_gainB,
            *force_control_d_gainB,
            *force_control_i_gainB,
            *force_control_spring_deflection;

// Demo Controller
Gtk::Scale *demo_ctrl_p_scale,
	   *demo_ctrl_d_scale,
	   *demo_ctrl_amplitude_scale,
	   *demo_ctrl_hip_p_scale,
	   *demo_ctrl_hip_d_scale;

Gtk::Button *demo_ctrl_start_btn,
	    *demo_ctrl_stop_btn;

Gtk::RadioButton *demo_ctrl_controller1_btn,
		 *demo_ctrl_controller2_btn,
		 *demo_ctrl_controller3_btn,
		 *demo_ctrl_controller4_btn,
		 *demo_ctrl_controller5_btn,
		 *demo_ctrl_controller6_btn;

int demo_ctrl_state;
#define DEMO_CTRL_STOPPED 0
#define DEMO_CTRL_STARTED 1

void start_demo(void);
void stop_demo(void);

// End Demo Controller


Gtk::DrawingArea *drawing_area;

Gtk::ProgressBar *motor_torqueA_progress_bar,
                 *motor_torqueB_progress_bar,
                 *motor_torqueHip_progress_bar,
                 *motor_velocityA_progress_bar,
                 *motor_velocityB_progress_bar,
                 *motor_velocityHip_progress_bar,
                 *spring_deflectionA_progress_bar,
                 *spring_deflectionB_progress_bar;

Gtk::CheckButton *log_file_chkbox;
Gtk::SpinButton *log_frequency_spin;
Gtk::FileChooserButton *log_file_chooser;

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
            *spring_deflection_B_entry;

Gtk::Button *restart_button,
            *enable_button,
            *disable_button;

Cairo::RefPtr<Cairo::Context> cc;
Gtk::Allocation drawing_allocation;

//FILE *logFile;

ros::ServiceClient datalog_client;

ros::Subscriber atrias_gui_sub;
ros::Publisher atrias_gui_pub;

atrias_msgs::atrias_data ad;
atrias_msgs::atrias_controller_requests cr;

atrias_controllers::data_subscriber_srv data_subscriber_srv;


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
