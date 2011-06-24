// Devin Koepl

#include <gui/atrias_gui.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "atrias_gui");	
	ros::NodeHandle n;	
	atrias_client = n.serviceClient<atrias_controllers::atrias_srv>("/gui_interface_srv");
	atrias_srv.request.command = CMD_DISABLE;
	atrias_srv.request.controller_requested = 0; // 0 for no controller, controllers need to be their own package I suppose.  Actually it would be better to move the gui inside the atrias package.

	Gtk::Main gtk(argc, argv);

	// Create the relative path to the Glade file.
	std::string glade_gui_path = std::string(argv[0]);
	
	glade_gui_path = glade_gui_path.substr(0, glade_gui_path.rfind("/bin"));
	glade_gui_path = glade_gui_path.append("/src/atrias_gui.glade");

	Glib::RefPtr<Gtk::Builder> gui = Gtk::Builder::create();
	try
	{
		gui->add_from_file(glade_gui_path);		
	}
	catch(const Glib::FileError& ex)
	{
		ROS_ERROR("File Error");
//			ROS_ERROR("FileError: %d", ex.what());
	}
	catch(const Gtk::BuilderError& ex)
	{
		ROS_ERROR("Builder Error");
//			ROS_ERROR("BuilderError: %d", ex.what());
	}

	// Grab pointers to GUI objects
	gui->get_widget("atrias_window", window);
	if(!window)
	{		
		ROS_ERROR("No ATRIAS Window");
	}
	
	gui->get_widget("controller_notebook", controller_notebook);

	gui->get_widget("motor_torqueA_hscale", motor_torqueA_hscale);
	gui->get_widget("motor_torqueB_hscale", motor_torqueB_hscale);

	gui->get_widget("motor_positionA_hscale", motor_positionA_hscale);
	gui->get_widget("motor_positionB_hscale", motor_positionB_hscale);
	gui->get_widget("p_motor_position_hscale", p_motor_position_hscale);
	gui->get_widget("d_motor_position_hscale", d_motor_position_hscale);

	gui->get_widget("leg_length_torque_hscale", leg_length_torque_hscale);
	gui->get_widget("leg_angle_torque_hscale", leg_angle_torque_hscale);
	gui->get_widget("p_leg_position_hscale", p_leg_position_hscale);
	gui->get_widget("d_leg_position_hscale", d_leg_position_hscale);

	gui->get_widget("leg_length_hscale", leg_length_hscale);
	gui->get_widget("leg_angle_hscale", leg_angle_hscale);

	gui->get_widget("leg_angle_amplitude_hscale", leg_angle_amplitude_hscale);
	gui->get_widget("leg_angle_frequency_hscale", leg_angle_frequency_hscale);
	gui->get_widget("leg_length_amplitude_hscale", leg_length_amplitude_hscale);
	gui->get_widget("leg_length_frequency_hscale", leg_length_frequency_hscale);
	gui->get_widget("p_sine_wave_hscale", p_sine_wave_hscale);
	gui->get_widget("d_sine_wave_hscale", d_sine_wave_hscale);

	gui->get_widget("raibert_desired_velocity_hscale", raibert_desired_velocity_hscale);
	gui->get_widget("raibert_desired_height_hscale", raibert_desired_height_hscale);
	gui->get_widget("raibert_hor_vel_gain_hscale", raibert_hor_vel_gain_hscale);
	gui->get_widget("raibert_leg_force_gain_hscale", raibert_leg_force_gain_hscale);
	gui->get_widget("raibert_leg_angle_gain_hscale", raibert_leg_angle_gain_hscale);
	gui->get_widget("raibert_stance_p_gain_hscale", raibert_stance_p_gain_hscale);
	gui->get_widget("raibert_stance_d_gain_hscale", raibert_stance_d_gain_hscale);
	gui->get_widget("raibert_stance_spring_threshold_hscale", raibert_stance_spring_threshold_hscale);
	gui->get_widget("raibert_state_label", raibert_state_label);
	gui->get_widget("raibert_preferred_leg_len_hscale", raibert_preferred_leg_len_hscale);
	gui->get_widget("raibert_flight_p_gain_hscale", raibert_flight_p_gain_hscale);
	gui->get_widget("raibert_flight_d_gain_hscale", raibert_flight_d_gain_hscale);
	gui->get_widget("raibert_flight_spring_threshold_hscale", raibert_flight_spring_threshold_hscale);

	gui->get_widget("drawing_area", drawing_area);

	gui->get_widget("motor_torqueA_progress_bar", motor_torqueA_progress_bar);
	gui->get_widget("motor_torqueB_progress_bar", motor_torqueB_progress_bar);

	gui->get_widget("log_file_chkbox", log_file_chkbox);
	gui->get_widget("log_file_chooser", log_file_chooser);

	gui->get_widget("hor_vel_label", hor_vel_label);
	gui->get_widget("height_label", height_label);

	gui->get_widget("restart_button", restart_button);
	gui->get_widget("enable_button", enable_button);
	gui->get_widget("disable_button", disable_button);

	raibert_state_label->set_label("Initializing");

	// Initialize GUI objects
	motor_torqueA_hscale->set_range(MTR_MIN_TRQ, MTR_MAX_TRQ);
	motor_torqueB_hscale->set_range(MTR_MIN_TRQ, MTR_MAX_TRQ);

	motor_positionA_hscale->set_range(-2.3562, 0.3054);
	motor_positionB_hscale->set_range(2.8362, 5.4978);
	p_motor_position_hscale->set_range(0., 1000.);
	d_motor_position_hscale->set_range(0., 50.);

	leg_length_torque_hscale->set_range(-10., 10.);
	leg_angle_torque_hscale->set_range(-10., 10.);

	leg_length_hscale->set_range(0., 1.);
	leg_angle_hscale->set_range(0., PI);
	p_leg_position_hscale->set_range(0., 1000.);
	d_leg_position_hscale->set_range(0., 100.);

	leg_angle_amplitude_hscale->set_range(0., 1.);
	leg_angle_frequency_hscale->set_range(0., 5.);
	leg_length_amplitude_hscale->set_range(0., 0.2);
	leg_length_frequency_hscale->set_range(0., 5.);
	p_sine_wave_hscale->set_range(0., 2000.);
	d_sine_wave_hscale->set_range(0., 50.);

	raibert_desired_velocity_hscale->set_range(-5., 5.);
	raibert_desired_height_hscale->set_range(0., 3.);
	raibert_hor_vel_gain_hscale->set_range(0., 10.);
	raibert_leg_force_gain_hscale->set_range(0., 1.);
	raibert_leg_angle_gain_hscale->set_range(0., 1.);
	raibert_stance_p_gain_hscale->set_range(0., 1000.);
	raibert_stance_d_gain_hscale->set_range(0., 50.);
	raibert_stance_spring_threshold_hscale->set_range(0., 1.);
	raibert_preferred_leg_len_hscale->set_range(0.7, 1.);
	raibert_flight_p_gain_hscale->set_range(0., 1000.);
	raibert_flight_d_gain_hscale->set_range(0., 50.);
	raibert_flight_spring_threshold_hscale->set_range(0., 1.);

	motor_torqueA_progress_bar->set_fraction(0.);
	motor_torqueB_progress_bar->set_fraction(0.);

	// Create the path to the data file with the last state of the gui gains.
	std::string gui_state_file = std::string(argv[0]);
	gui_state_file = gui_state_file.substr(0, gui_state_file.rfind("/bin"));
	gui_state_file = gui_state_file.append("/src/gui_last_state.dat");

	// Set the gains in the gui according to their last values.
  /*FILE *gui_state_fp = fopen (gui_state_file.c_str(), "r");
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
	if ( fscanf(gui_state_fp, "Leg Position P Gain: %f\n", &read_val) < 1 )
	{
		ROS_WARN("Gui state file read error.");
	}
	p_leg_position_hscale->set_value(read_val);
	if ( fscanf(gui_state_fp, "Leg Position D Gain: %f\n", &read_val) < 1 )
	{
		ROS_WARN("Gui state file read error.");
	}
	d_leg_position_hscale->set_value(read_val);
	if ( fscanf(gui_state_fp, "Sine Wave P Gain: %f\n", &read_val) < 1 )
	{
		ROS_WARN("Gui state file read error.");
	}
	p_sine_wave_hscale->set_value(read_val);
	if ( fscanf(gui_state_fp, "Sine Wave D Gain: %f\n", &read_val) < 1 )
	{
		ROS_WARN("Gui state file read error.");
	}
	d_sine_wave_hscale->set_value(read_val);
	if ( fscanf(gui_state_fp, "Raibert Velocity Gain: %f\n", &read_val) < 1 )
	{
		ROS_WARN("Gui state file read error.");
	}
	raibert_hor_vel_gain_hscale->set_value(read_val);
	if ( fscanf(gui_state_fp, "Raibert Height Gain: %f\n", &read_val) < 1 )
	{
		ROS_WARN("Gui state file read error.");
	}
	raibert_leg_force_gain_hscale->set_value(read_val);
	if ( fscanf(gui_state_fp, "Raibert Leg TD Angle Gain: %f\n", &read_val) < 1 )
	{
		ROS_WARN("Gui state file read error.");
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

  fclose (gui_state_fp);*/

	raibert_desired_velocity_hscale->set_value(0.364);
	raibert_desired_height_hscale->set_value(1.2);
	raibert_hor_vel_gain_hscale->set_value(0.162);
	raibert_leg_force_gain_hscale->set_value(0.225);
	raibert_leg_angle_gain_hscale->set_value(0.15);
	raibert_stance_p_gain_hscale->set_value(350.);
	raibert_stance_d_gain_hscale->set_value(10.);
	raibert_stance_spring_threshold_hscale->set_value(0.05);
	raibert_preferred_leg_len_hscale->set_value(0.97);
	raibert_flight_p_gain_hscale->set_value(250.);
	raibert_flight_d_gain_hscale->set_value(12.);
	raibert_flight_spring_threshold_hscale->set_value(0.022);

	drawing_allocation = drawing_area->get_allocation();

	// Connect buttons to functions.
	log_file_chkbox->signal_toggled().connect( sigc::ptr_fun(log_chkbox_toggled) );
	restart_button->signal_clicked().connect( sigc::ptr_fun( restart_robot ) );
	enable_button->signal_clicked().connect( sigc::ptr_fun( enable_motors ) );
	disable_button->signal_clicked().connect( sigc::ptr_fun( disable_motors ) );

	controller_notebook->signal_switch_page().connect( sigc::ptr_fun(switch_controllers) );

	sigc::connection conn = Glib::signal_timeout().connect(sigc::ptr_fun(poke_controller), 100); // 50 is the timeout in milliseconds

	gtk.run(*window);

	// Store the final state of the gui.
	/*gui_state_fp = fopen(gui_state_file.c_str(), "w");

	fprintf(gui_state_fp, "Motor Position P Gain: %f\n", p_motor_position_hscale->get_value());
	fprintf(gui_state_fp, "Motor Position D Gain: %f\n", d_motor_position_hscale->get_value());
	fprintf(gui_state_fp, "Leg Position P Gain: %f\n", p_leg_position_hscale->get_value());
	fprintf(gui_state_fp, "Leg Position D Gain: %f\n", d_leg_position_hscale->get_value());
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

void log_chkbox_toggled( void )
{
	if ( log_file_chkbox->get_active() )
	{
		// Open the log file for data logging.
		log_file_fp = fopen(log_file_chooser->get_filename().c_str(), "w");
		fprintf(log_file_fp, "time, body_ang, mtr_angA, mtr_angB, leg_angA, leg_angB, mtr_trqA, mtr_trqB, hor_vel, height\n");
	}
	else
	{
		// Close the log file.
		fclose(log_file_fp);
	}
}

void restart_robot( void )
{
	if ( atrias_srv.request.command == CMD_DISABLE )
	{
		atrias_srv.request.command = CMD_RESTART;
		atrias_srv.request.controller_requested = NO_CONTROLLER;
		controller_notebook->set_current_page( NO_CONTROLLER );
	}
}

void enable_motors( void )
{
	atrias_srv.request.command = CMD_RUN;
}

void disable_motors( void )
{
	atrias_srv.request.command = CMD_DISABLE;
}

void switch_controllers(GtkNotebookPage* page, guint page_num)
{
	if ( atrias_srv.request.command == CMD_DISABLE )
		atrias_srv.request.controller_requested = page_num;
	else
		controller_notebook->set_current_page( atrias_srv.request.controller_requested );
}

bool poke_controller( void )
{
	char buffer[20];

	switch (atrias_srv.request.controller_requested)
	{
		case NO_CONTROLLER:	
			//MTR_TRQ_CONTROLLER_DATA(&(atrias_srv.request.control_data.elems))->mtr_trqA = 0.;			
			break;
		case MOTOR_TORQUE_CONTROLLER:
			((MtrTrqControllerData *)(&(atrias_srv.request.control_data.elems)))->mtr_trqA = motor_torqueA_hscale->get_value();
			((MtrTrqControllerData *)(&(atrias_srv.request.control_data.elems)))->mtr_trqB = motor_torqueB_hscale->get_value();
			break;
		case MOTOR_POSITION_CONTROLLER:
			((MtrPosControllerData *)(&(atrias_srv.request.control_data.elems)))->mtr_angA = motor_positionA_hscale->get_value();
			((MtrPosControllerData *)(&(atrias_srv.request.control_data.elems)))->mtr_angB = motor_positionB_hscale->get_value();	
			((MtrPosControllerData *)(&(atrias_srv.request.control_data.elems)))->p_gain = p_motor_position_hscale->get_value();
			((MtrPosControllerData *)(&(atrias_srv.request.control_data.elems)))->d_gain = d_motor_position_hscale->get_value();
			break;
		case LEG_TORQUE_CONTROLLER:
			((LegTrqControllerData *)(&(atrias_srv.request.control_data.elems)))->leg_ang_trq = leg_angle_torque_hscale->get_value();
			((LegTrqControllerData *)(&(atrias_srv.request.control_data.elems)))->leg_len_trq = leg_length_torque_hscale->get_value();
			break;
		case LEG_POSITION_CONTROLLER:
			((LegPosControllerData *)(&(atrias_srv.request.control_data.elems)))->leg_len = leg_length_hscale->get_value();
			((LegPosControllerData *)(&(atrias_srv.request.control_data.elems)))->leg_ang =	leg_angle_hscale->get_value();
			((LegPosControllerData *)(&(atrias_srv.request.control_data.elems)))->p_gain = p_leg_position_hscale->get_value();
			((LegPosControllerData *)(&(atrias_srv.request.control_data.elems)))->d_gain = d_leg_position_hscale->get_value();
			break;
		case SINE_WAVE_CONTROLLER:
			((SinWaveControllerData *)(&(atrias_srv.request.control_data.elems)))->leg_ang_frq = leg_angle_frequency_hscale->get_value();
			((SinWaveControllerData *)(&(atrias_srv.request.control_data.elems)))->leg_ang_amp = leg_angle_amplitude_hscale->get_value();
			((SinWaveControllerData *)(&(atrias_srv.request.control_data.elems)))->leg_len_frq = leg_length_frequency_hscale->get_value();;
			((SinWaveControllerData *)(&(atrias_srv.request.control_data.elems)))->leg_len_amp = leg_length_amplitude_hscale->get_value();;
			((SinWaveControllerData *)(&(atrias_srv.request.control_data.elems)))->p_gain = p_sine_wave_hscale->get_value();
			((SinWaveControllerData *)(&(atrias_srv.request.control_data.elems)))->d_gain = d_sine_wave_hscale->get_value();
			break;
		case RAIBERT_CONTROLLER:
			((RaibertControllerData *)(&(atrias_srv.request.control_data.elems)))->des_hor_vel = raibert_desired_velocity_hscale->get_value();
			((RaibertControllerData *)(&(atrias_srv.request.control_data.elems)))->des_hop_ht = raibert_desired_height_hscale->get_value();
			((RaibertControllerData *)(&(atrias_srv.request.control_data.elems)))->hor_vel_gain = raibert_hor_vel_gain_hscale->get_value();
			((RaibertControllerData *)(&(atrias_srv.request.control_data.elems)))->hop_ht_gain = raibert_leg_force_gain_hscale->get_value();
			((RaibertControllerData *)(&(atrias_srv.request.control_data.elems)))->leg_ang_gain = raibert_leg_angle_gain_hscale->get_value();
			((RaibertControllerData *)(&(atrias_srv.request.control_data.elems)))->stance_p_gain = raibert_stance_p_gain_hscale->get_value();
			((RaibertControllerData *)(&(atrias_srv.request.control_data.elems)))->stance_d_gain = raibert_stance_d_gain_hscale->get_value();
			((RaibertControllerData *)(&(atrias_srv.request.control_data.elems)))->stance_spring_threshold = raibert_stance_spring_threshold_hscale->get_value();
			((RaibertControllerData *)(&(atrias_srv.request.control_data.elems)))->preferred_leg_len = raibert_preferred_leg_len_hscale->get_value();			
			((RaibertControllerData *)(&(atrias_srv.request.control_data.elems)))->flight_p_gain = raibert_flight_p_gain_hscale->get_value();
			((RaibertControllerData *)(&(atrias_srv.request.control_data.elems)))->flight_d_gain = raibert_flight_d_gain_hscale->get_value();
			((RaibertControllerData *)(&(atrias_srv.request.control_data.elems)))->flight_spring_threshold = raibert_flight_spring_threshold_hscale->get_value();

			// Set the state label.
			if ( atrias_srv.response.status == CMD_DISABLE )
				raibert_state_label->set_label("Disabled");
			else
				if ( ((RaibertControllerState *)(&(atrias_srv.response.control_state.elems)))->in_flight )
					raibert_state_label->set_label("Flight");	
				else
					raibert_state_label->set_label("Stance");
	}		

	// Check to see if we are supposed to be logging data.
	if ( log_file_chkbox->get_active() )
		fprintf(log_file_fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", atrias_srv.response.time, atrias_srv.response.body_angle, 
			atrias_srv.response.motor_angleA, atrias_srv.response.motor_angleB, atrias_srv.response.leg_angleA, atrias_srv.response.leg_angleB,
			atrias_srv.response.motor_torqueA, atrias_srv.response.motor_torqueB, atrias_srv.response.hor_vel, atrias_srv.response.height);

	if ( atrias_client.call( atrias_srv ) )
		draw_leg();

	if ( atrias_srv.request.command == CMD_RESTART )
		atrias_srv.request.command = CMD_DISABLE;

	// Move the sliders if the controller is disabled.
	if ( atrias_srv.response.status == CMD_DISABLE )
	{
		motor_positionA_hscale->set_value(atrias_srv.response.motor_angleA);
		motor_positionB_hscale->set_value(atrias_srv.response.motor_angleB);
	}

	// Update the motor torque progress bars.
	motor_torqueA_progress_bar->set_fraction( MIN ( ABS(atrias_srv.response.motor_torqueA), MTR_MAX_TRQ ) / MTR_MAX_TRQ);
	motor_torqueB_progress_bar->set_fraction( MIN ( ABS(atrias_srv.response.motor_torqueB), MTR_MAX_TRQ ) / MTR_MAX_TRQ);

	// Update the boom stuff indicators.
	sprintf(buffer, "%.4f", atrias_srv.response.hor_vel);
	hor_vel_label->set_label(buffer);	
	sprintf(buffer, "%.4f", atrias_srv.response.height);
	height_label->set_label(buffer);	
	
	return true;
}

void draw_leg()
{
	float segment_length = 125.;
	float short_segment_length = 100.;
	float motor_radius = 70.;

	float start_x = 260.;
	float start_y = 100.;

	drawing_area->get_window()->clear();
	
	cr = drawing_area->get_window()->create_cairo_context();
	
	cr->set_line_width(12.0);

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
}
