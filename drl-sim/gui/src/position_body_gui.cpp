// Devin Koepl
// Colan Dray

#include <gui/position_body_gui.h>

int main(int argc, char **argv)
{
    // Standard ROS Initializations
    ros::init(argc, argv, "atrias_gui");
    ros::NodeHandle n;
    simulation_client = n.serviceClient<drl_plugins::position_body_srv>("/position_body_srv");
    simulation_srv.request.hold_robot = true;
    simulation_srv.request.pause_simulation = false;
    simulation_srv.request.desired_pose.position.x = 0.;
    simulation_srv.request.desired_pose.position.y = 0.;
    simulation_srv.request.desired_pose.position.z = 1.;

    Gtk::Main gtk(argc, argv);
    
    // Create the relative path to the Glade file.
    std::string glade_gui_path = std::string(argv[0]);
    glade_gui_path = glade_gui_path.substr(0, glade_gui_path.rfind("/bin"));
    glade_gui_path = glade_gui_path.append("/src/position_body_gui.glade");

    Glib::RefPtr<Gtk::Builder> gui = Gtk::Builder::create();
    try
    {
        gui->add_from_file(glade_gui_path);        
    }
    catch(const Glib::FileError& ex)
    {
        ROS_ERROR("File Error"); //ROS_ERROR("FileError: %d", ex.what());
    }
    catch(const Gtk::BuilderError& ex)
    {
        ROS_ERROR("Builder Error"); //ROS_ERROR("BuilderError: %d", ex.what());
    }

    // Grab pointers to GUI objects
    gui->get_widget("window", window);
    if(!window)
    {        
        ROS_ERROR("No GUI Window");
    }
    gui->get_widget("x_translation_checkbutton", x_translation_checkbutton);
    //gui->get_widget("x_rotation_checkbutton", x_rotation_checkbutton);
    gui->get_widget("x_position_hscale", x_position_hscale);
    
    gui->get_widget("y_translation_checkbutton", y_translation_checkbutton);
    //gui->get_widget("y_rotation_checkbutton", y_rotation_checkbutton);
    gui->get_widget("y_position_hscale", y_position_hscale);
    
    gui->get_widget("z_translation_checkbutton", z_translation_checkbutton);
    //gui->get_widget("z_rotation_checkbutton", z_rotation_checkbutton);    
    gui->get_widget("z_position_hscale", z_position_hscale);

    gui->get_widget("pause_play_button", pause_play_button);
    gui->get_widget("hold_release_button", hold_release_button);
    gui->get_widget("get_position_button", get_position_button);

    // Initialize GUI objects
    x_position_hscale->set_range(-100., 100.);
    y_position_hscale->set_range(-100., 100.);
    z_position_hscale->set_range(0.5, 1.5);
    
    x_position_hscale->set_value(0.0);
    y_position_hscale->set_value(0.0);
    z_position_hscale->set_value(1.0);

    // Wait for a maximum of 10000 ms simulation to come online.
    /*if (!ros::service::waitForService("/position_body_srv", 10000))
    {
        ROS_ERROR("No simulation detected.");
    }*/

    // Connect buttons to functions.
    pause_play_button->signal_toggled().connect( sigc::ptr_fun(pause_play) );
    hold_release_button->signal_toggled().connect( sigc::ptr_fun(hold_release) );
    get_position_button->signal_clicked().connect( sigc::ptr_fun(get_position) );

    x_position_hscale->signal_value_changed().connect( sigc::ptr_fun(desired_pose_changed) );
    y_position_hscale->signal_value_changed().connect( sigc::ptr_fun(desired_pose_changed) );
    z_position_hscale->signal_value_changed().connect( sigc::ptr_fun(desired_pose_changed) );
    x_translation_checkbutton->signal_toggled().connect( sigc::ptr_fun(desired_pose_changed) );
    y_translation_checkbutton->signal_toggled().connect( sigc::ptr_fun(desired_pose_changed) );
    z_translation_checkbutton->signal_toggled().connect( sigc::ptr_fun(desired_pose_changed) );

    gtk.run(*window);

  return 0;
}

void pause_play()
{
    simulation_srv.request.pause_simulation = !simulation_srv.request.pause_simulation;
    while( !simulation_client.call(simulation_srv) );
}

void hold_release()
{
    simulation_srv.request.hold_robot = !simulation_srv.request.hold_robot;
    while( !simulation_client.call(simulation_srv) );
}

void get_position()
{
    while( !simulation_client.call(simulation_srv) );
    desired_pose = simulation_srv.response.actual_pose;
    x_position_hscale->set_value(desired_pose.position.x);
    y_position_hscale->set_value(desired_pose.position.y);
    z_position_hscale->set_value(desired_pose.position.z + 0.025); // +0.025 is a crude hack to fix gravity error
}

void desired_pose_changed()
{
    if( x_translation_checkbutton->get_active() )
    {
        simulation_srv.request.desired_pose.position.x = x_position_hscale->get_value();
    }
    if( y_translation_checkbutton->get_active() )
    {
        simulation_srv.request.desired_pose.position.y = y_position_hscale->get_value();
    }
    if( z_translation_checkbutton->get_active() )
    {
        simulation_srv.request.desired_pose.position.z = z_position_hscale->get_value();
    }
    while( !simulation_client.call(simulation_srv) );
}
