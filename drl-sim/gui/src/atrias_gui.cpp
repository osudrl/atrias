//! @file position_body_gui.cpp
//! @brief The interface between the GUI and the robot control logic.
//! @author Devin Koepl

#include <gui/atrias_gui.h>
#include <cmath>

//! @brief Initializes the GUI and controls.
//! @param argc An integer that is one more than the number of command line arguments.
//! @param argv An array of character pointers containing the command line arguments.
//! @return Returns zero upon successful completion, non-zero if an error occured.

#define DELTA_Y (0.5*sin(atrias_srv.response.motor_angleB) + 0.5*sin(atrias_srv.response.motor_angleA))
#define DELTA_X (-0.5*cos(atrias_srv.response.motor_angleB) - 0.5*cos(atrias_srv.response.motor_angleA))
#define LEG_LENGTH sqrt(DELTA_Y*DELTA_Y + DELTA_X*DELTA_X)
#define LEG_ANGLE ((0.0*PI) - (atan2(DELTA_Y,DELTA_X)))

int main (int argc, char **argv) {
    ros::init(argc, argv, "atrias_gui");
    ros::NodeHandle nh;

    atrias_client = nh.serviceClient<atrias_controllers::atrias_srv>("gui_interface_srv");
    atrias_srv.request.command = CMD_DISABLE;
    atrias_srv.request.controller_requested = 0;
    // 0 for no controller, controllers need to be their own package I suppose.  Actually it would be better to move the gui inside the atrias package.

    // Subscribe to service named "data_subscriber_srv".
    datalog_client = nh.serviceClient<atrias_controllers::data_subscriber_srv>("data_subscriber_srv");

    Gtk::Main gtk(argc, argv);

    // Create the relative path to the Glade file.
    std::string glade_gui_path = std::string(argv[0]);
    red_image_path = std::string(argv[0]);
    green_image_path = std::string(argv[0]);

    red_image_path = green_image_path = glade_gui_path = glade_gui_path.substr(0, glade_gui_path.rfind("/bin"));
    red_image_path.append("/src/media/red.png");
    green_image_path.append("/src/media/green.png");
    glade_gui_path.append("/src/atrias_gui.glade");

    Glib::RefPtr<Gtk::Builder> gui = Gtk::Builder::create();
    try {
        gui->add_from_file(glade_gui_path);
    }
    catch (const Glib::FileError& ex) {
        ROS_ERROR("File Error");
        //			ROS_ERROR("FileError: %d", ex.what());
    }
    catch (const Gtk::BuilderError& ex) {
        ROS_ERROR("Builder Error");
        //			ROS_ERROR("BuilderError: %d", ex.what());
    }

    // Grab pointers to GUI objects
    gui->get_widget("atrias_window", window);
    if (!window) {
        ROS_ERROR("No ATRIAS Window");
    }
    gui->get_widget("controller_notebook", controller_notebook);

    gui->get_widget("motor_torqueA_hscale", motor_torqueA_hscale);
    gui->get_widget("motor_torqueB_hscale", motor_torqueB_hscale);
    gui->get_widget("motor_torque_hip_hscale", motor_torque_hip_hscale);

    gui->get_widget("motor_positionA_hscale", motor_positionA_hscale);
    gui->get_widget("motor_positionB_hscale", motor_positionB_hscale);
    gui->get_widget("p_motor_position_hscale", p_motor_position_hscale);
    gui->get_widget("d_motor_position_hscale", d_motor_position_hscale);

    gui->get_widget("leg_length_torque_hscale", leg_length_torque_hscale);
    gui->get_widget("leg_angle_torque_hscale", leg_angle_torque_hscale);
    gui->get_widget("p_leg_position_hscale", p_leg_position_hscale);
    gui->get_widget("d_leg_position_hscale", d_leg_position_hscale);

    gui->get_widget("hip_position_ang", hip_position_ang);
    gui->get_widget("hip_position_p", hip_position_p);
    gui->get_widget("hip_position_d", hip_position_d);
    //gui->get_widget("p_leg_position_spin", p_leg_position_spin);
    //gui->get_widget("d_leg_position_spin", d_leg_position_spin);

    gui->get_widget("leg_length_hscale", leg_length_hscale);
    gui->get_widget("leg_angle_hscale", leg_angle_hscale);

    gui->get_widget("leg_angle_amplitude_hscale", leg_angle_amplitude_hscale);
    gui->get_widget("leg_angle_frequency_hscale", leg_angle_frequency_hscale);
    gui->get_widget("leg_length_amplitude_hscale", leg_length_amplitude_hscale);
    gui->get_widget("leg_length_frequency_hscale", leg_length_frequency_hscale);
    gui->get_widget("p_sine_wave_hscale", p_sine_wave_hscale);
    gui->get_widget("d_sine_wave_hscale", d_sine_wave_hscale);

    gui->get_widget("raibert_state_label", raibert_state_label);
    gui->get_widget("raibert_desired_velocity_hscale", raibert_desired_velocity_hscale);
    gui->get_widget("raibert_hor_vel_gain_hscale", raibert_hor_vel_gain_hscale);
    gui->get_widget("raibert_leg_angle_gain_hscale", raibert_leg_angle_gain_hscale);
    gui->get_widget("raibert_stance_p_gain_hscale", raibert_stance_p_gain_hscale);
    gui->get_widget("raibert_stance_d_gain_hscale", raibert_stance_d_gain_hscale);
    gui->get_widget("raibert_stance_spring_threshold_hscale", raibert_stance_spring_threshold_hscale);
    gui->get_widget("raibert_desired_height_hscale", raibert_desired_height_hscale);
    gui->get_widget("raibert_leg_force_gain_hscale", raibert_leg_force_gain_hscale);
    gui->get_widget("raibert_preferred_leg_len_hscale", raibert_preferred_leg_len_hscale);
    gui->get_widget("raibert_flight_p_gain_hscale", raibert_flight_p_gain_hscale);
    gui->get_widget("raibert_flight_d_gain_hscale", raibert_flight_d_gain_hscale);
    gui->get_widget("raibert_flight_spring_threshold_hscale", raibert_flight_spring_threshold_hscale);
    gui->get_widget("raibert_stance_hip_p_gain", raibert_stance_hip_p_gain);
    gui->get_widget("raibert_stance_hip_d_gain", raibert_stance_hip_d_gain);
    gui->get_widget("raibert_flight_hip_p_gain", raibert_flight_hip_p_gain);
    gui->get_widget("raibert_flight_hip_d_gain", raibert_flight_hip_d_gain);

    gui->get_widget("raibert_desired_velocity_spinbutton", raibert_desired_velocity_spinbutton);
    gui->get_widget("raibert_hor_vel_gain_spinbutton", raibert_hor_vel_gain_spinbutton);
    gui->get_widget("raibert_leg_angle_gain_spinbutton", raibert_leg_angle_gain_spinbutton);
    gui->get_widget("raibert_stance_p_gain_spinbutton", raibert_stance_p_gain_spinbutton);
    gui->get_widget("raibert_stance_d_gain_spinbutton", raibert_stance_d_gain_spinbutton);
    gui->get_widget("raibert_stance_spring_threshold_spinbutton", raibert_stance_spring_threshold_spinbutton);
    gui->get_widget("raibert_desired_height_spinbutton", raibert_desired_height_spinbutton);
    gui->get_widget("raibert_leg_force_gain_spinbutton", raibert_leg_force_gain_spinbutton);
    gui->get_widget("raibert_preferred_leg_len_spinbutton", raibert_preferred_leg_len_spinbutton);
    gui->get_widget("raibert_flight_p_gain_spinbutton", raibert_flight_p_gain_spinbutton);
    gui->get_widget("raibert_flight_d_gain_spinbutton", raibert_flight_d_gain_spinbutton);
    gui->get_widget("raibert_flight_spring_threshold_spinbutton", raibert_flight_spring_threshold_spinbutton);

    gui->get_widget("hubicki_state_label", hubicki_state_label);
    gui->get_widget("hubicki_desired_velocity_hscale", hubicki_desired_velocity_hscale);
    gui->get_widget("hubicki_hor_vel_gain_hscale", hubicki_hor_vel_gain_hscale);
    gui->get_widget("hubicki_leg_angle_gain_hscale", hubicki_leg_angle_gain_hscale);
    gui->get_widget("hubicki_stance_p_gain_hscale", hubicki_stance_p_gain_hscale);
    gui->get_widget("hubicki_stance_d_gain_hscale", hubicki_stance_d_gain_hscale);
    gui->get_widget("hubicki_stance_spring_threshold_hscale", hubicki_stance_spring_threshold_hscale);
    gui->get_widget("hubicki_desired_height_hscale", hubicki_desired_height_hscale);
    gui->get_widget("hubicki_leg_force_gain_hscale", hubicki_leg_force_gain_hscale);
    gui->get_widget("hubicki_preferred_leg_len_hscale", hubicki_preferred_leg_len_hscale);
    gui->get_widget("hubicki_flight_p_gain_hscale", hubicki_flight_p_gain_hscale);
    gui->get_widget("hubicki_flight_d_gain_hscale", hubicki_flight_d_gain_hscale);
    gui->get_widget("hubicki_flight_spring_threshold_hscale", hubicki_flight_spring_threshold_hscale);
    gui->get_widget("hubicki_stance_hip_p_gain", hubicki_stance_hip_p_gain);
    gui->get_widget("hubicki_stance_hip_d_gain", hubicki_stance_hip_d_gain);
    gui->get_widget("hubicki_flight_hip_p_gain", hubicki_flight_hip_p_gain);
    gui->get_widget("hubicki_flight_hip_d_gain", hubicki_flight_hip_d_gain);

    gui->get_widget("hubicki_desired_velocity_spinbutton", hubicki_desired_velocity_spinbutton);
    gui->get_widget("hubicki_hor_vel_gain_spinbutton", hubicki_hor_vel_gain_spinbutton);
    gui->get_widget("hubicki_leg_angle_gain_spinbutton", hubicki_leg_angle_gain_spinbutton);
    gui->get_widget("hubicki_stance_p_gain_spinbutton", hubicki_stance_p_gain_spinbutton);
    gui->get_widget("hubicki_stance_d_gain_spinbutton", hubicki_stance_d_gain_spinbutton);
    gui->get_widget("hubicki_stance_spring_threshold_spinbutton", hubicki_stance_spring_threshold_spinbutton);
    gui->get_widget("hubicki_desired_height_spinbutton", hubicki_desired_height_spinbutton);
    gui->get_widget("hubicki_leg_force_gain_spinbutton", hubicki_leg_force_gain_spinbutton);
    gui->get_widget("hubicki_preferred_leg_len_spinbutton", hubicki_preferred_leg_len_spinbutton);
    gui->get_widget("hubicki_flight_p_gain_spinbutton", hubicki_flight_p_gain_spinbutton);
    gui->get_widget("hubicki_flight_d_gain_spinbutton", hubicki_flight_d_gain_spinbutton);
    gui->get_widget("hubicki_flight_spring_threshold_spinbutton", hubicki_flight_spring_threshold_spinbutton);

    gui->get_widget("test_motors_status_image", test_motors_status_image);
    gui->get_widget("test_flight_status_image", test_flight_status_image);
    gui->get_widget("test_label_1", test_label);
    test_label->set_text("Flight KP");
    gui->get_widget("test_label_2", test_label);
    test_label->set_text("Flight KD");
    gui->get_widget("test_label_3", test_label);
    test_label->set_text("Stance KP");
    gui->get_widget("test_label_4", test_label);
    test_label->set_text("Stance KD");
    gui->get_widget("test_label_5", test_label);
    test_label->set_text("Desired Length Long");
    gui->get_widget("test_label_6", test_label);
    test_label->set_text("Desired Length Short");
    gui->get_widget("test_label_7", test_label);
    test_label->set_text("Toe Switch Threshold");
    gui->get_widget("test_label_8", test_label);
    test_label->set_text("Spring Deflection Threshold");
    gui->get_widget("test_label_9", test_label);
    test_label->set_text("Spring Deflection A");
    gui->get_widget("test_label_0", test_label);
    test_label->set_text("Spring Deflection B");
    gui->get_widget("test_hscale_1", test_slider_flightKP);
    gui->get_widget("test_hscale_2", test_slider_flightKD);
    gui->get_widget("test_hscale_3", test_slider_stanceKP);
    gui->get_widget("test_hscale_4", test_slider_stanceKD);
    gui->get_widget("test_hscale_5", test_slider_desiredLengthLong);
    gui->get_widget("test_hscale_6", test_slider_desiredLengthShort);
    gui->get_widget("test_hscale_7", test_slider_toeSwitchThreshold);
    gui->get_widget("test_hscale_8", test_slider_springDeflectionThreshold);
    gui->get_widget("test_hscale_9", test_slider_springDeflectionA);
    gui->get_widget("test_hscale_0", test_slider_springDeflectionB);

    /*
    gui->get_widget("grizzle_flight_threshold_hscale", grizzle_flight_threshold_hscale);
    gui->get_widget("grizzle_stance_threshold_hscale", grizzle_stance_threshold_hscale);
    gui->get_widget("grizzle_motor_gain_hscale", grizzle_motor_gain_hscale);
     */

    gui->get_widget("force_control_p_gainA", force_control_p_gainA);
    gui->get_widget("force_control_d_gainA", force_control_d_gainA);
    gui->get_widget("force_control_i_gainA", force_control_i_gainA);
    gui->get_widget("force_control_p_gainB", force_control_p_gainB);
    gui->get_widget("force_control_d_gainB", force_control_d_gainB);
    gui->get_widget("force_control_i_gainB", force_control_i_gainB);
    gui->get_widget("force_control_spring_deflection", force_control_spring_deflection);

    gui->get_widget("drawing_area", drawing_area);

    gui->get_widget("motor_torqueA_progress_bar", motor_torqueA_progress_bar);
    gui->get_widget("motor_torqueB_progress_bar", motor_torqueB_progress_bar);

    gui->get_widget("log_file_chkbox", log_file_chkbox);
    gui->get_widget("log_frequency_spin", log_frequency_spin);
    gui->get_widget("log_file_chooser", log_file_chooser);

    gui->get_widget("xPosDisplay", xPosDisplay);
    gui->get_widget("yPosDisplay", yPosDisplay);
    gui->get_widget("zPosDisplay", zPosDisplay);
    gui->get_widget("xVelDisplay", xVelDisplay);
    gui->get_widget("yVelDisplay", yVelDisplay);
    gui->get_widget("zVelDisplay", zVelDisplay);

    gui->get_widget("torqueADisplay", torqueADisplay);
    gui->get_widget("torqueBDisplay", torqueBDisplay);

    gui->get_widget("spring_deflection_A_entry", spring_deflection_A_entry);
    gui->get_widget("spring_deflection_B_entry", spring_deflection_B_entry);
    gui->get_widget("spring_deflectionA_progress_bar", spring_deflectionA_progress_bar);
    gui->get_widget("spring_deflectionB_progress_bar", spring_deflectionB_progress_bar);

    gui->get_widget("velocityADisplay", velocityADisplay);
    gui->get_widget("velocityBDisplay", velocityBDisplay);

    gui->get_widget("motor_velocityA_progress_bar", motor_velocityA_progress_bar);
    gui->get_widget("motor_velocityB_progress_bar", motor_velocityB_progress_bar);
    
    gui->get_widget("restart_button", restart_button);
    gui->get_widget("enable_button", enable_button);
    gui->get_widget("disable_button", disable_button);

    /*
     * This block is for the Medulla Status section.
     */
    gui->get_widget("MedullaA_TempA",MedullaA_TempA);
    gui->get_widget("MedullaA_TempB",MedullaA_TempB);
    gui->get_widget("MedullaA_TempC",MedullaA_TempC);
    gui->get_widget("MedullaA_VLogic",MedullaA_VLogic);
    gui->get_widget("MedullaA_VMotor",MedullaA_VMotor);
    gui->get_widget("MedullaA_Error",MedullaA_Error);

    gui->get_widget("MedullaB_TempA",MedullaB_TempA);
    gui->get_widget("MedullaB_TempB",MedullaB_TempB);
    gui->get_widget("MedullaB_TempC",MedullaB_TempC);
    gui->get_widget("MedullaB_VLogic",MedullaB_VLogic);
    gui->get_widget("MedullaB_VMotor",MedullaB_VMotor);
    gui->get_widget("MedullaB_Error",MedullaB_Error);


    raibert_state_label->set_label("Initializing");
    hubicki_state_label->set_label("Initializing");

    /* 
     * #region Initialize GUI objects
     *
     */
    motor_torqueA_hscale->set_range(MTR_MIN_TRQ, MTR_MAX_TRQ);
    motor_torqueB_hscale->set_range(MTR_MIN_TRQ, MTR_MAX_TRQ);
    motor_torque_hip_hscale->set_range(MTR_MIN_TRQ, MTR_MAX_TRQ);

    motor_positionA_hscale->set_range(-2.3562, 0.3054);
    motor_positionB_hscale->set_range(2.8362, 5.4978);
    p_motor_position_hscale->set_range(0., 50000.);
    d_motor_position_hscale->set_range(0., 5000.);

    leg_length_torque_hscale->set_range(-10., 10.);
    leg_angle_torque_hscale->set_range(-10., 10.);

    leg_length_hscale->set_range(0.5, 1.);
    leg_angle_hscale->set_range(0.29, 2.85);
    p_leg_position_hscale->set_range(0., 10000.);
    //p_leg_position_spin->set_range(0., 1000.);
    //p_leg_position_spin->set_increments(.05, 1.);
    d_leg_position_hscale->set_range(0., 300.);
    //d_leg_position_spin->set_increments(.05, 1.);
    //d_leg_position_spin->set_range(0., 1000.);

    hip_position_ang ->set_range(-0.209, 0.209);
    hip_position_p->set_range(0., 10000.);
    hip_position_d->set_range(0., 300.);

    log_frequency_spin->set_range(100, 10000);
    log_frequency_spin->set_increments(100, 500);
    log_frequency_spin->set_value(100);

    leg_angle_amplitude_hscale->set_range(0., 1.0);
    leg_angle_frequency_hscale->set_range(0., 20.);
    leg_length_amplitude_hscale->set_range(0., 0.2);
    leg_length_frequency_hscale->set_range(0., 20.);
    p_sine_wave_hscale->set_range(0., 10000.);
    d_sine_wave_hscale->set_range(0., 100.);


	/* Raibert tab */
	// HScales
    raibert_desired_velocity_hscale->set_range(-5., 5.);
    raibert_hor_vel_gain_hscale->set_range(0., 10.);
    raibert_leg_angle_gain_hscale->set_range(0., 1.);
    raibert_stance_p_gain_hscale->set_range(0., 6000.);
    raibert_stance_d_gain_hscale->set_range(0., 150.);
    raibert_stance_spring_threshold_hscale->set_range(0., 1.);
	raibert_desired_height_hscale->set_range(0., 3.);
    raibert_leg_force_gain_hscale->set_range(0., 1.);
    raibert_preferred_leg_len_hscale->set_range(0.7, 1.);
    raibert_flight_p_gain_hscale->set_range(0., 1000.);
    raibert_flight_d_gain_hscale->set_range(0., 50.);
    raibert_flight_spring_threshold_hscale->set_range(0., 1.);
    raibert_stance_hip_p_gain->set_range(0,5000.0);
    raibert_stance_hip_d_gain->set_range(0,100.0);
    raibert_flight_hip_p_gain->set_range(0,5000.0);
    raibert_flight_hip_d_gain->set_range(0,100.0);

	// Spinbuttons
    raibert_desired_velocity_spinbutton->set_range(-5., 5.);
    raibert_hor_vel_gain_spinbutton->set_range(0., 10.);
    raibert_leg_angle_gain_spinbutton->set_range(0., 1.);
    raibert_stance_p_gain_spinbutton->set_range(0., 6000.);
    raibert_stance_d_gain_spinbutton->set_range(0., 150.);
    raibert_stance_spring_threshold_spinbutton->set_range(0., 0.4);
	raibert_desired_height_spinbutton->set_range(0., 3.);
    raibert_leg_force_gain_spinbutton->set_range(0., 1.);
    raibert_preferred_leg_len_spinbutton->set_range(0.7, 1.);
    raibert_flight_p_gain_spinbutton->set_range(0., 4000.);
    raibert_flight_d_gain_spinbutton->set_range(0., 150.);
    raibert_flight_spring_threshold_spinbutton->set_range(0., 0.4);


	/* Hubicki tab */
	// HScales
    hubicki_desired_velocity_hscale->set_range(-5., 5.);
    hubicki_hor_vel_gain_hscale->set_range(0., 10.);
    hubicki_leg_angle_gain_hscale->set_range(0., 1.);
    hubicki_stance_p_gain_hscale->set_range(0., 6000.);
    hubicki_stance_d_gain_hscale->set_range(0., 150.);
    hubicki_stance_spring_threshold_hscale->set_range(0., 1.);
	hubicki_desired_height_hscale->set_range(0., 3.);
    hubicki_leg_force_gain_hscale->set_range(0., 1.);
    hubicki_preferred_leg_len_hscale->set_range(0.7, 1.);
    hubicki_flight_p_gain_hscale->set_range(0., 1000.);
    hubicki_flight_d_gain_hscale->set_range(0., 50.);
    hubicki_flight_spring_threshold_hscale->set_range(0., 1.);
    hubicki_stance_hip_p_gain->set_range(0,5000.0);
    hubicki_stance_hip_d_gain->set_range(0,100.0);
    hubicki_flight_hip_p_gain->set_range(0,5000.0);
    hubicki_flight_hip_d_gain->set_range(0,100.0);

	// Spinbuttons
    hubicki_desired_velocity_spinbutton->set_range(-5., 5.);
    hubicki_hor_vel_gain_spinbutton->set_range(0., 10.);
    hubicki_leg_angle_gain_spinbutton->set_range(0., 1.);
    hubicki_stance_p_gain_spinbutton->set_range(0., 6000.);
    hubicki_stance_d_gain_spinbutton->set_range(0., 150.);
    hubicki_stance_spring_threshold_spinbutton->set_range(0., 0.4);
	hubicki_desired_height_spinbutton->set_range(0., 3.);
    hubicki_leg_force_gain_spinbutton->set_range(0., 1.);
    hubicki_preferred_leg_len_spinbutton->set_range(0.7, 1.);
    hubicki_flight_p_gain_spinbutton->set_range(0., 4000.);
    hubicki_flight_d_gain_spinbutton->set_range(0., 150.);
    hubicki_flight_spring_threshold_spinbutton->set_range(0., 0.4);


	/* Test tab */
    test_slider_flightKP->set_range(0.0, 1000.0);
    test_slider_flightKD->set_range(0.0, 100.0);
    test_slider_stanceKP->set_range(0.0, 1000.0);
    test_slider_stanceKD->set_range(0.0, 100.0);
    test_slider_desiredLengthLong->set_range(0.0, 1.0);
    test_slider_desiredLengthShort->set_range(0.0, 1.0);
    test_slider_toeSwitchThreshold->set_range(0.0, 2.0);
    test_slider_springDeflectionThreshold->set_range(0.0, 5.0);
    test_slider_springDeflectionA->set_range(-100.0, 100.0);
    test_slider_springDeflectionB->set_range(-100.0, 100.0);

    motor_torqueA_progress_bar->set_fraction(0.);
    motor_torqueB_progress_bar->set_fraction(0.);

    motor_velocityA_progress_bar->set_fraction(0.);
    motor_velocityB_progress_bar->set_fraction(0.);
    
    test_motors_status_image->set(red_image_path);
    test_flight_status_image->set(red_image_path);

    /* Force Control Tab */
    force_control_p_gainA->set_range(0.0,100000.0);
    force_control_d_gainA->set_range(0.0,300.0);
    force_control_i_gainA->set_range(0.0,30000.0);
    force_control_p_gainB->set_range(0.0,100000.0);
    force_control_d_gainB->set_range(0.0,300.0);
    force_control_i_gainB->set_range(0.0,30000.0);
    force_control_spring_deflection->set_range(-0.2,0.2);

    // Create the path to the data file with the last state of the gui gains.
    std::string gui_state_file = std::string(argv[0]);
    gui_state_file = gui_state_file.substr(0, gui_state_file.rfind("/bin"));
    gui_state_file = gui_state_file.append("/src/gui_last_state.dat");

    // Set the gains in the gui according to their last values.
   /* FILE *gui_state_fp = fopen (gui_state_file.c_str(), "r");
          float read_val;

          if ( fscanf(gui_state_fp, "Motor Position P Gain: %f\n", &read_val) < 1 )
          {
                  ROS_WARN("Gui state file read error.");
          }
          p_motor_position_hscale->set_value(read_val);
          if ( fscanf(gui_state_fp, "Motor Position D Gain: %f\n", &read_val) < 1 )
          {
                  ROS_WARN("Gui state file read error.");
          }
          d_motor_position_hscale->set_value(read_val);
          if ( fscanf(zVelDisplayui_state_fp, "Leg Position P Gain: %f\n", &read_val) < 1 )
          {
                  ROS_WARN("Gui state file read error.");
          }
          p_leg_position_hscale->set_value(read_val);
          if ( fscanf(gui_state_fp, "Leg Position D Gain: %f\n", &read_val) < 1 )
          {
                  ROS_WARN("Gui state file read error.");
          }
          d_leg_position_hscale->set_value(read_val);
          if ( fscanf(gui_state_launch-prefix="xterm -e gdb --args"fp, "Sine Wave P Gain: %f\n", &read_val) < 1 )
          {
                  ROS_WARN("Gui state file read error.");
          }
          p_sine_wave_hscale->set_value(read_val);
          if ( fscanf(gui_state_fp, "Sine Wave D Gain: %f\n", &read_val) < 1 )
          {
                  ROS_WARN("Gui state file read error.");
          }
          d_sine_wave_hscale->set_value(read_val);
          ROS_INFO("initialized!");scale->set_value(read_val);
          if ( fscanf(gui_state_fp, "Raibert Velocity Gain: %f\n", &read_val) < 1 )
          {
                  ROS_WARN("Gui state file read error.");
          }
          raibert_hor_vel_gain_hscale->set_value(read_val);
          if ( fscanf(gui_state_fp, "Raibert Height Gain: %f\n", &read_val) < 1 )
          {
          ROS_INFO("check 3 good!");
                  ROS_WARN("Gui state file read error.");
          }
          raibert_leg_force_gain_hscale->set_value(read_val);
          if ( fscanf(gui_state_fp, "Raibert Leg TD Angle Gain: %f\n", &read_val) < 1 )
          {
                  VROS_WARN("Gui state file read error.");
          }
          raibert_hor_vel_gain_hscale->set_value(read_val);
          if ( fscanf(gui_state_fp, "Raibert P Gain: %f\n", &read_val) < 1 )
          {
                  ROS_WARN("Gui state file read error.");
          }
          raibert_stance_p_gain_hscale->set_value(read_val);
          if ( fscanf(gui_state_fp, "Raibert D Gain: %f\n", &read_val) < 1 )
          {
                  ROS_WARN("Gui state file read error.");
          }
          raibert_stance_d_gain_hscale->set_value(read_val);
          if ( fscanf(gui_state_fp, "Raibert Spring Deflection Threshold: %f\n", &read_val) < 1 )
          {
                  ROS_WARN("Gui state file read error.");
          }
          raibert_stance_spring_threshold_hscale->set_value(read_val);

    fclose (gui_state_fp);
*/
	// Leg position tab default values
    leg_length_hscale->set_value(0.9);
    leg_angle_hscale->set_value(PI/2);
    p_leg_position_hscale->set_value(50);
    d_leg_position_hscale->set_value(2.5);

	// HScale default values for Raibert controller
    raibert_desired_velocity_hscale->set_value(0.);
    raibert_hor_vel_gain_hscale->set_value(0.0);
    raibert_leg_angle_gain_hscale->set_value(0.0);
    raibert_stance_p_gain_hscale->set_value(0.0);
    raibert_stance_d_gain_hscale->set_value(0.0);
    raibert_stance_spring_threshold_hscale->set_value(0.0);
    raibert_stance_hip_p_gain->set_value(2000.0);
    raibert_stance_hip_d_gain->set_value(20.0);
    raibert_desired_height_hscale->set_value(0.0);
    raibert_leg_force_gain_hscale->set_value(0.0);
    raibert_preferred_leg_len_hscale->set_value(0.0);
    raibert_flight_p_gain_hscale->set_value(0.0);
    raibert_flight_d_gain_hscale->set_value(0.0);
    raibert_flight_spring_threshold_hscale->set_value(0.0);
    raibert_flight_hip_p_gain->set_value(0.0);
    raibert_flight_hip_d_gain->set_value(0.0);

	// Spinbutton default values for Raibert controller
    raibert_desired_velocity_spinbutton->set_value(0.25);
    raibert_hor_vel_gain_spinbutton->set_value(0.25);
    raibert_leg_angle_gain_spinbutton->set_value(0.125);
    raibert_stance_p_gain_spinbutton->set_value(6000.);
    raibert_stance_d_gain_spinbutton->set_value(120.);
    raibert_stance_spring_threshold_spinbutton->set_value(0.075);
    raibert_desired_height_spinbutton->set_value(1.5);
    raibert_leg_force_gain_spinbutton->set_value(0.3);
    raibert_preferred_leg_len_spinbutton->set_value(0.9);
    raibert_flight_p_gain_spinbutton->set_value(750.);
    raibert_flight_d_gain_spinbutton->set_value(150.);
    raibert_flight_spring_threshold_spinbutton->set_value(0.035);

	// HScale default values for Hubicki controller
    hubicki_desired_velocity_hscale->set_value(0.);
    hubicki_hor_vel_gain_hscale->set_value(0.0);
    hubicki_leg_angle_gain_hscale->set_value(0.0);
    hubicki_stance_p_gain_hscale->set_value(0.0);
    hubicki_stance_d_gain_hscale->set_value(0.0);
    hubicki_stance_spring_threshold_hscale->set_value(0.0);
    hubicki_stance_hip_p_gain->set_value(2000.0);
    hubicki_stance_hip_d_gain->set_value(20.0);
    hubicki_desired_height_hscale->set_value(0.0);
    hubicki_leg_force_gain_hscale->set_value(0.0);
    hubicki_preferred_leg_len_hscale->set_value(0.0);
    hubicki_flight_p_gain_hscale->set_value(0.0);
    hubicki_flight_d_gain_hscale->set_value(0.0);
    hubicki_flight_spring_threshold_hscale->set_value(0.0);
    hubicki_flight_hip_p_gain->set_value(0.0);
    hubicki_flight_hip_d_gain->set_value(0.0);

	// Spinbutton default values for Hubicki controller
    hubicki_desired_velocity_spinbutton->set_value(0.25);
    hubicki_hor_vel_gain_spinbutton->set_value(0.25);
    hubicki_leg_angle_gain_spinbutton->set_value(0.125);
    hubicki_stance_p_gain_spinbutton->set_value(6000.);
    hubicki_stance_d_gain_spinbutton->set_value(120.);
    hubicki_stance_spring_threshold_spinbutton->set_value(0.075);
    hubicki_desired_height_spinbutton->set_value(1.5);
    hubicki_leg_force_gain_spinbutton->set_value(0.3);
    hubicki_preferred_leg_len_spinbutton->set_value(0.9);
    hubicki_flight_p_gain_spinbutton->set_value(750.);
    hubicki_flight_d_gain_spinbutton->set_value(150.);
    hubicki_flight_spring_threshold_spinbutton->set_value(0.035);


    test_slider_flightKP->set_value(100.0);
    test_slider_flightKD->set_value(10.0);
    test_slider_stanceKP->set_value(350.0);
    test_slider_stanceKD->set_value(15.0);
    test_slider_desiredLengthLong->set_value(0.99);
    test_slider_desiredLengthShort->set_value(0.85);
    test_slider_toeSwitchThreshold->set_value(0.02);
    test_slider_springDeflectionThreshold->set_value(1.0);
    test_slider_springDeflectionA->set_value(0.0);
    test_slider_springDeflectionB->set_value(0.0);

    /*
     * #end Initialize GUI region 
     *
     */

    drawing_allocation = drawing_area->get_allocation();
    /*
     * Connect buttons to functions.
     */
    log_file_chkbox->signal_toggled().connect(sigc::ptr_fun(log_chkbox_toggled));
    restart_button->signal_clicked().connect(sigc::ptr_fun(restart_robot));
    enable_button->signal_clicked().connect(sigc::ptr_fun(enable_motors));
    disable_button->signal_clicked().connect(sigc::ptr_fun(disable_motors));
    controller_notebook->signal_switch_page().connect(sigc::ptr_fun(switch_controllers));

    sigc::connection conn = Glib::signal_timeout().connect(sigc::ptr_fun(poke_controller), 100); // 50 is the timeout in milliseconds

    ROS_INFO("GUI: Running.");
    gtk.run(*window);

    // Store the final state of the gui.
/*    gui_state_fp = fopen(gui_state_file.c_str(), "w");

    fprintf(gui_state_fp, "Motor Position P Gain: %f\n", p_motor_position_hscale->get_value());
    fprintf(gui_state_fp, "Motor Position D Gain: %f\n", d_motor_position_hscale->get_value());
    fprintf(gui_state_fp, "Leg Position P Gain: %f\n", p_leg_position_hscale->get_value());
    fprintf(("zVelDisplay", zVgui_state_fp, "Leg Position D Gain: %f\n", d_leg_position_hscale->get_value());
    fprintf(gui_state_fp, "Sine Wave P Gain: %f\n", p_sine_wave_hscale->get_value());
    fprintf(gui_state_fp, "Sine Wave D Gain: %f\n", d_sine_wave_hscale->get_value());
    fprintf(gui_state_fp, "Raibert Velocity Gain: %f\n", raibert_hor_vel_gain_hscale->get_value());
    fprintf(gui_state_fp, "Raibert Height Gain: %f\n", raibert_leg_force_gain_hscale->get_value());
    fprintf(gui_state_fp, "Raibert Leg TD Angle Gain: %f\n", raibert_leg_angle_gain_hscale->get_value());
    fprintf(gui_state_fp, "Raibert P Gain: %f\n", raibert_stance_p_gain_hscale->get_value());
    fprintf(gui_state_fp, "Raibert D Gain: %f\n", raibert_stance_d_gain_hscale->get_value());
    fprintf(gui_state_fp, "Raibert Spring Deflection Threshold: %f\n", raibert_stance_spring_threshold_hscale->get_value());

    fclose(gui_state_fp);*/
    return 0;
}

//! @brief Announce for a log file to be created.
// TODO: data_subscriber_srv should be named something like desktop_datalog_srv.
void log_chkbox_toggled (void) {
    if (log_file_chkbox->get_active()) {   // If checkbox is checked...
        ROS_INFO("GUI: Sending log enable request.");
        if (log_file_chooser->get_filename() != "") {
            data_subscriber_srv.request.logfilename = log_file_chooser->get_filename();   // Specify name to use for logfile.
        }
        data_subscriber_srv.request.isLogging = true;
    }
    else {
        ROS_INFO("GUI: Sending log disable request.");
        log_file_chooser->unselect_all();
        data_subscriber_srv.request.isLogging = false;
    }

    datalog_client.call(data_subscriber_srv);   // Call the service with request and receive response.

    if (data_subscriber_srv.response.logfilename != "") {
        log_file_chooser->set_filename(data_subscriber_srv.response.logfilename);   // Set FileChooserButton filename to response.
    }
}

//! @brief restarts the robot.
void restart_robot (void) {
    if (atrias_srv.request.command == CMD_DISABLE) {
        atrias_srv.request.command = CMD_RESTART;
        atrias_srv.request.controller_requested = NO_CONTROLLER;
        controller_notebook->set_current_page(NO_CONTROLLER);
    }
}

//! @brief Enables the motors of the robot.
void enable_motors (void) {
    atrias_srv.request.command = CMD_RUN;
}

//! @brief Disables the motors of the robot. 
void disable_motors (void) {
    atrias_srv.request.command = CMD_DISABLE;
    test_motors_status_image->set(red_image_path);
}

//! @brief Change the active controller.
void switch_controllers (GtkNotebookPage* page, guint page_num) {
    if (atrias_srv.request.command == CMD_DISABLE)
        atrias_srv.request.controller_requested = page_num;
    else
        controller_notebook->set_current_page(atrias_srv.request.controller_requested);

    leg_length_hscale->set_value(LEG_LENGTH);
    leg_angle_hscale->set_value(LEG_ANGLE);

}

//! @brief Probes and updates the active controller then sends updated commands to the robot.
bool poke_controller (void) {
    char buffer[20];
    switch (atrias_srv.request.controller_requested) {
        case NO_CONTROLLER:
            //MTR_TRQ_CONTROLLER_DATA(&(atrias_srv.request.control_data.elems))->mtr_trqA = 0.;
            break;
        case MOTOR_TORQUE_CONTROLLER:
            ((MtrTrqControllerData *) (&(atrias_srv.request.control_data.elems)))->mtr_trqA = motor_torqueA_hscale->get_value();
            ((MtrTrqControllerData *) (&(atrias_srv.request.control_data.elems)))->mtr_trqB = motor_torqueB_hscale->get_value();
            ((MtrTrqControllerData *) (&(atrias_srv.request.control_data.elems)))->mtr_trq_hip = motor_torque_hip_hscale->get_value();
            break;
        case MOTOR_POSITION_CONTROLLER:
            ((MtrPosControllerData *) (&(atrias_srv.request.control_data.elems)))->mtr_angA = motor_positionA_hscale->get_value();
            ((MtrPosControllerData *) (&(atrias_srv.request.control_data.elems)))->mtr_angB = motor_positionB_hscale->get_value();
            ((MtrPosControllerData *) (&(atrias_srv.request.control_data.elems)))->p_gain = p_motor_position_hscale->get_value();
            ((MtrPosControllerData *) (&(atrias_srv.request.control_data.elems)))->d_gain = d_motor_position_hscale->get_value();
            break;
        case LEG_TORQUE_CONTROLLER:
            ((LegTrqControllerData *) (&(atrias_srv.request.control_data.elems)))->leg_ang_trq = leg_angle_torque_hscale->get_value();
            ((LegTrqControllerData *) (&(atrias_srv.request.control_data.elems)))->leg_len_trq = leg_length_torque_hscale->get_value();
            break;
        case LEG_POSITION_CONTROLLER:
            ((LegPosControllerData *) (&(atrias_srv.request.control_data.elems)))->leg_len = leg_length_hscale->get_value();
            ((LegPosControllerData *) (&(atrias_srv.request.control_data.elems)))->leg_ang = leg_angle_hscale->get_value();
            /*if (fabs(p_leg_position_spin->get_value() - last_p_gain) > .00001)
            {
                  p_leg_position_hscale->set_value(p_leg_position_spin->get_value());
            }
            else if (fabs(p_leg_position_hscale->get_value() - last_p_gain) > .00001)
            {
                  p_leg_position_spin->set_value(p_leg_position_hscale->get_value());
            }
            if (fabs(d_leg_position_spin->get_value() - last_d_gain) > .00001)
            {
                  d_leg_position_hscale->set_value(d_leg_position_spin->get_value());
            }
            else if (fabs(d_leg_position_hscale->get_value() - last_d_gain) > .00001)
            {
                  d_leg_position_spin->set_value(d_leg_position_hscale->get_value());
            }*/
            ((LegPosControllerData *) (&(atrias_srv.request.control_data.elems)))->p_gain = p_leg_position_hscale->get_value();
            ((LegPosControllerData *) (&(atrias_srv.request.control_data.elems)))->d_gain = d_leg_position_hscale->get_value();
            ((LegPosControllerData *) (&(atrias_srv.request.control_data.elems)))->hip_ang = hip_position_ang->get_value();
            ((LegPosControllerData *) (&(atrias_srv.request.control_data.elems)))->hip_p_gain = hip_position_p->get_value();
            ((LegPosControllerData *) (&(atrias_srv.request.control_data.elems)))->hip_d_gain = hip_position_d->get_value();
            break;
        case SINE_WAVE_CONTROLLER:
            ((SinWaveControllerData *) (&(atrias_srv.request.control_data.elems)))->leg_ang_frq = leg_angle_frequency_hscale->get_value();
            ((SinWaveControllerData *) (&(atrias_srv.request.control_data.elems)))->leg_ang_amp = leg_angle_amplitude_hscale->get_value();
            ((SinWaveControllerData *) (&(atrias_srv.request.control_data.elems)))->leg_len_frq = leg_length_frequency_hscale->get_value();
            ((SinWaveControllerData *) (&(atrias_srv.request.control_data.elems)))->leg_len_amp = leg_length_amplitude_hscale->get_value();
            ((SinWaveControllerData *) (&(atrias_srv.request.control_data.elems)))->p_gain = p_sine_wave_hscale->get_value();
            ((SinWaveControllerData *) (&(atrias_srv.request.control_data.elems)))->d_gain = d_sine_wave_hscale->get_value();
            break;
        case RAIBERT_CONTROLLER:
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->des_hor_vel = raibert_desired_velocity_spinbutton->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->des_hop_ht = raibert_desired_height_spinbutton->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->hor_vel_gain = raibert_hor_vel_gain_spinbutton->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->hop_ht_gain = raibert_leg_force_gain_spinbutton->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->leg_ang_gain = raibert_leg_angle_gain_spinbutton->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->stance_p_gain = raibert_stance_p_gain_spinbutton->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->stance_d_gain = raibert_stance_d_gain_spinbutton->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->stance_spring_threshold = raibert_stance_spring_threshold_spinbutton->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->preferred_leg_len = raibert_preferred_leg_len_spinbutton->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->flight_p_gain = raibert_flight_p_gain_spinbutton->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->flight_d_gain = raibert_flight_d_gain_spinbutton->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->flight_spring_threshold = raibert_flight_spring_threshold_spinbutton->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->stance_hip_p_gain = raibert_stance_hip_p_gain->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->stance_hip_d_gain = raibert_stance_hip_d_gain->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->flight_hip_p_gain = raibert_flight_hip_p_gain->get_value();
            ((RaibertControllerData *) (&(atrias_srv.request.control_data.elems)))->flight_hip_d_gain = raibert_flight_hip_d_gain->get_value();

            // Set the state label.
            if (atrias_srv.response.status == CMD_DISABLE)
                raibert_state_label->set_label("Disabled");
            else
                if (((RaibertControllerState *) (&(atrias_srv.response.control_state.elems)))->in_flight)
                raibert_state_label->set_label("Flight");
            else
                raibert_state_label->set_label("Stance");
            break;
        case HUBICKI_CONTROLLER:
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->des_hor_vel = hubicki_desired_velocity_spinbutton->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->des_hop_ht = hubicki_desired_height_spinbutton->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->hor_vel_gain = hubicki_hor_vel_gain_spinbutton->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->hop_ht_gain = hubicki_leg_force_gain_spinbutton->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->leg_ang_gain = hubicki_leg_angle_gain_spinbutton->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->stance_p_gain = hubicki_stance_p_gain_spinbutton->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->stance_d_gain = hubicki_stance_d_gain_spinbutton->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->stance_spring_threshold = hubicki_stance_spring_threshold_spinbutton->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->preferred_leg_len = hubicki_preferred_leg_len_spinbutton->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->flight_p_gain = hubicki_flight_p_gain_spinbutton->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->flight_d_gain = hubicki_flight_d_gain_spinbutton->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->flight_spring_threshold = hubicki_flight_spring_threshold_spinbutton->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->stance_hip_p_gain = hubicki_stance_hip_p_gain->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->stance_hip_d_gain = hubicki_stance_hip_d_gain->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->flight_hip_p_gain = hubicki_flight_hip_p_gain->get_value();
            ((HubickiControllerData *) (&(atrias_srv.request.control_data.elems)))->flight_hip_d_gain = hubicki_flight_hip_d_gain->get_value();

            // Set the state label.
            if (atrias_srv.response.status == CMD_DISABLE)
                hubicki_state_label->set_label("Disabled");
            else
                if (((HubickiControllerState *) (&(atrias_srv.response.control_state.elems)))->in_flight)
                hubicki_state_label->set_label("Flight");
            else
                hubicki_state_label->set_label("Stance");
            break;
        case TEST_CONTROLLER:
            ((TestControllerData *) (&(atrias_srv.request.control_data.elems)))->flightKP = test_slider_flightKP->get_value();
            ((TestControllerData *) (&(atrias_srv.request.control_data.elems)))->flightKD = test_slider_flightKD->get_value();
            ((TestControllerData *) (&(atrias_srv.request.control_data.elems)))->stanceKP = test_slider_stanceKP->get_value();
            ((TestControllerData *) (&(atrias_srv.request.control_data.elems)))->stanceKD = test_slider_stanceKD->get_value();
            ((TestControllerData *) (&(atrias_srv.request.control_data.elems)))->desiredLengthLong = test_slider_desiredLengthLong->get_value();
            ((TestControllerData *) (&(atrias_srv.request.control_data.elems)))->desiredLengthShort = test_slider_desiredLengthShort->get_value();
            ((TestControllerData *) (&(atrias_srv.request.control_data.elems)))->toeSwitchThreshold = test_slider_toeSwitchThreshold->get_value();
            ((TestControllerData *) (&(atrias_srv.request.control_data.elems)))->springDeflectionThreshold = test_slider_springDeflectionThreshold->get_value();

            test_slider_springDeflectionA->set_value(((TestControllerState *) (&(atrias_srv.response.control_state.elems)))->springDeflectionAverageAOld);
            test_slider_springDeflectionB->set_value(((TestControllerState *) (&(atrias_srv.response.control_state.elems)))->springDeflectionAverageBOld);

            if (((TestControllerState *) (&(atrias_srv.response.control_state.elems)))->currentState > 0) {
                test_motors_status_image->set(green_image_path);
            }
            else {
                test_motors_status_image->set(red_image_path);
            }
            if (((TestControllerState *) (&(atrias_srv.response.control_state.elems)))->currentState == 0) {
                test_flight_status_image->set(green_image_path);
            }
            else {
                test_flight_status_image->set(red_image_path);
            }
            break;
	case FORCE_CONTROLLER:
	    ((ForceControllerData *) (&(atrias_srv.request.control_data.elems)))->p_gainA = force_control_p_gainA->get_value();
            ((ForceControllerData *) (&(atrias_srv.request.control_data.elems)))->d_gainA = force_control_d_gainA->get_value();
            ((ForceControllerData *) (&(atrias_srv.request.control_data.elems)))->i_gainA = force_control_i_gainA->get_value();
	    ((ForceControllerData *) (&(atrias_srv.request.control_data.elems)))->p_gainB = force_control_p_gainB->get_value();
            ((ForceControllerData *) (&(atrias_srv.request.control_data.elems)))->d_gainB = force_control_d_gainB->get_value();
            ((ForceControllerData *) (&(atrias_srv.request.control_data.elems)))->i_gainB = force_control_i_gainB->get_value();
            ((ForceControllerData *) (&(atrias_srv.request.control_data.elems)))->spring_deflection = force_control_spring_deflection->get_value();
	    break;
    }

    if (atrias_client.call(atrias_srv))
        draw_leg();

    if (atrias_srv.request.command == CMD_RESTART)
        atrias_srv.request.command = CMD_DISABLE;

    // Move the sliders if the controller is disabled.
    if (atrias_srv.response.status == CMD_DISABLE) {
        motor_positionA_hscale->set_value(atrias_srv.response.motor_angleA);
        motor_positionB_hscale->set_value(atrias_srv.response.motor_angleB);
    }

    // Update the motor torque progress bars and displays.
    sprintf(buffer, "%0.4f", atrias_srv.response.motor_torqueA);
    torqueADisplay->set_text(buffer);
    sprintf(buffer, "%0.4f", atrias_srv.response.motor_torqueB);
    torqueBDisplay->set_text(buffer);

    motor_torqueA_progress_bar->set_fraction(MIN(ABS(atrias_srv.response.motor_torqueA), MTR_MAX_TRQ) / MTR_MAX_TRQ);
    motor_torqueB_progress_bar->set_fraction(MIN(ABS(atrias_srv.response.motor_torqueB), MTR_MAX_TRQ) / MTR_MAX_TRQ);

    motor_velocityA_progress_bar->set_fraction(MIN(ABS(atrias_srv.response.motor_velocityA * 477.45), 1327) / 1327);
    motor_velocityB_progress_bar->set_fraction(MIN(ABS(atrias_srv.response.motor_velocityB * 477.45), 1327) / 1327);
    sprintf(buffer, "%0.2f", atrias_srv.response.motor_velocityA * 477.45);
    velocityADisplay->set_text(buffer);
    sprintf(buffer, "%0.2f", atrias_srv.response.motor_velocityB * 477.45);
    velocityBDisplay->set_text(buffer);
   
    // Update spring deflection displays.
    sprintf(buffer, "%0.8f", atrias_srv.response.motor_angleA - atrias_srv.response.leg_angleA);
    spring_deflection_A_entry->set_text(buffer);
    sprintf(buffer, "%0.8f", atrias_srv.response.motor_angleB - atrias_srv.response.leg_angleB);
    spring_deflection_B_entry->set_text(buffer);

    //replace with a macro?
    spring_deflectionA_progress_bar->set_fraction(
            log10(abs(atrias_srv.response.motor_angleA - atrias_srv.response.leg_angleA) + 1)
            / log10(21));

    spring_deflectionB_progress_bar->set_fraction(
            log10(abs(atrias_srv.response.motor_angleA - atrias_srv.response.leg_angleA) + 1)
            / log10(21));

    // Update the boom stuff indicators.
    sprintf(buffer, "%0.4f", atrias_srv.response.xPosition);
    xPosDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f", atrias_srv.response.yPosition);
    yPosDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f", atrias_srv.response.zPosition);
    zPosDisplay->set_text(buffer);

    sprintf(buffer, "%0.4f", atrias_srv.response.xVelocity);
    xVelDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f", atrias_srv.response.yVelocity);
    yVelDisplay->set_text(buffer);
    sprintf(buffer, "%0.4f", atrias_srv.response.zVelocity);
    zVelDisplay->set_text(buffer);

    /* Update Medulla status */
    sprintf(buffer,"%0.2f",atrias_srv.response.thermistorA[0]);
    MedullaA_TempA->set_text(buffer);

    sprintf(buffer,"%0.2f",atrias_srv.response.thermistorA[1]);
    MedullaA_TempB->set_text(buffer);

    sprintf(buffer,"%0.2f",atrias_srv.response.thermistorA[2]);
    MedullaA_TempC->set_text(buffer);

    sprintf(buffer,"%0.1f",atrias_srv.response.motorVoltageA);
    MedullaA_VMotor->set_text(buffer);

    sprintf(buffer,"%0.1f",atrias_srv.response.logicVoltageA);
    MedullaA_VLogic->set_text(buffer);

    sprintf(buffer,"%0.2f",atrias_srv.response.thermistorB[0]);
    MedullaB_TempA->set_text(buffer);

    sprintf(buffer,"%0.2f",atrias_srv.response.thermistorB[1]);
    MedullaB_TempB->set_text(buffer);

    sprintf(buffer,"%0.2f",atrias_srv.response.thermistorB[2]);
    MedullaB_TempC->set_text(buffer);

    sprintf(buffer,"%0.1f",atrias_srv.response.motorVoltageB);
    MedullaB_VMotor->set_text(buffer);

    sprintf(buffer,"%0.1f",atrias_srv.response.logicVoltageB);
    MedullaB_VLogic->set_text(buffer);

//	ROS_INFO("MedulaA_Error: %d",atrias_srv.response.medullaStatusA);
    if (atrias_srv.response.medullaStatusA)
    {
        Glib::ustring error;
	printf("%d\n",atrias_srv.response.medullaStatusA);
        if (atrias_srv.response.medullaStatusA & STATUS_ESTOP)
            error += "EStop Pressed ";
        if (atrias_srv.response.medullaStatusA & STATUS_LIMITSW)
            error += "Limit Switch ";
        if (atrias_srv.response.medullaStatusA & STATUS_OVER_TEMP)
            error += "Motor Over Temperature ";
        if (atrias_srv.response.medullaStatusA & STATUS_MOTOR_OUT_OF_RANGE)
            error += "Motor Out of Range ";
        if (atrias_srv.response.medullaStatusA & STATUS_MOTOR_CTRL_DISABLE)
            error += "Motor Control Disabled ";
        if (atrias_srv.response.medullaStatusA & STATUS_MOTOR_VOLTAGE_LOW)
            error += "Motor Voltage Low ";
        if (atrias_srv.response.medullaStatusA & STATUS_LOGIC_VOLTAGE_LOW)
            error += "Logic Voltage Low ";
        if (atrias_srv.response.medullaStatusA & STATUS_ENCODER_ERROR)
            error += "Encoder Error ";

        MedullaA_Error->set_text(error);
    }

    if (atrias_srv.response.medullaStatusB)
    {
        Glib::ustring error;
        if (atrias_srv.response.medullaStatusB & STATUS_ESTOP)
            error += "EStop Pressed ";
        if (atrias_srv.response.medullaStatusB & STATUS_LIMITSW)
            error += "Limit Switch ";
        if (atrias_srv.response.medullaStatusB & STATUS_OVER_TEMP)
            error += "Motor Over Temperature ";
        if (atrias_srv.response.medullaStatusB & STATUS_MOTOR_OUT_OF_RANGE)
            error += "Motor Out of Range ";
        if (atrias_srv.response.medullaStatusB & STATUS_MOTOR_CTRL_DISABLE)
            error += "Motor Control Disabled ";
        if (atrias_srv.response.medullaStatusB & STATUS_MOTOR_VOLTAGE_LOW)
            error += "Motor Voltage Low ";
        if (atrias_srv.response.medullaStatusB & STATUS_LOGIC_VOLTAGE_LOW)
            error += "Logic Voltage Low ";
        if (atrias_srv.response.medullaStatusB & STATUS_ENCODER_ERROR)
            error += "Encoder Error ";

        MedullaB_Error->set_text(error);
    }

    /* End Medulla status*/

    return true;
}

//! @brief Draws the four legs of Atrias in the simulation (the carrot).
void draw_leg () {
    float segment_length = 125.;
    float short_segment_length = 100.;
    float motor_radius = 70.;

    float start_x = 125.;
    float start_y = 125.;

    drawing_area->get_window()->clear();

    cr = drawing_area->get_window()->create_cairo_context();


    cr->set_line_width(12.0);
	
	// Draw the leg
    cr->move_to(start_x, start_y);
    // OSU orange 216, 90, 26
    cr->set_source_rgb(0.8471, 0.3529, 0.1020);
    //cr->set_source_rgb(0.8, 0.0, 0.0);
    // A
    cr->rel_line_to(short_segment_length * cos(atrias_srv.response.leg_angleA), -short_segment_length * sin(atrias_srv.response.leg_angleA));
    // C
    cr->rel_line_to(segment_length * cos(atrias_srv.response.leg_angleB), -segment_length * sin(atrias_srv.response.leg_angleB));
    // B
    cr->move_to(start_x, start_y);
    cr->rel_line_to(segment_length * cos(atrias_srv.response.leg_angleB), -segment_length * sin(atrias_srv.response.leg_angleB));
    // D
    cr->rel_line_to(segment_length * cos(atrias_srv.response.leg_angleA), -segment_length * sin(atrias_srv.response.leg_angleA));
    cr->stroke();

    // Draw the motors
    cr->set_source_rgb(0.0, 0.8, 0.0);
    // A
    cr->move_to(start_x, start_y);
    cr->rel_line_to(motor_radius * cos(atrias_srv.response.motor_angleA), -motor_radius * sin(atrias_srv.response.motor_angleA));
    cr->move_to(start_x, start_y);
    cr->rel_line_to(-motor_radius * cos(atrias_srv.response.motor_angleA), motor_radius * sin(atrias_srv.response.motor_angleA));
    // B
    cr->move_to(start_x, start_y);
    cr->rel_line_to(motor_radius * cos(atrias_srv.response.motor_angleB), -motor_radius * sin(atrias_srv.response.motor_angleB));
    cr->move_to(start_x, start_y);
    cr->rel_line_to(-motor_radius * cos(atrias_srv.response.motor_angleB), motor_radius * sin(atrias_srv.response.motor_angleB));
    cr->stroke();
}

//! @brief Draw a simple, live plot of the last 30 seconds.
void draw_plot () {

}

