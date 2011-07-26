// Devin Koepl
// Colan Dray

#include <gui/position_body_gui.h>

int main(int argc, char **argv)
{
    // ROS Initializations
    ros::init(argc, argv, "atrias_gui");
    ros::NodeHandle n;
    simulation_client = n.serviceClient<drl_plugins::position_body_srv>("/position_body_srv");
    reset_client = n.serviceClient<std_srvs::Empty>("gazebo/reset_simulation");
    simulation_srv.request.hold_robot = true;
    simulation_srv.request.pause_simulation = false;
    simulation_srv.request.desired_pose.position.x = 0.;
    simulation_srv.request.desired_pose.position.y = 0.;
    simulation_srv.request.desired_pose.position.z = 1.;

    // Gtk Initializations
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
        ROS_ERROR("File Error");
    }
    catch(const Gtk::BuilderError& ex)
    {
        ROS_ERROR("Builder Error");
    }

    // Grab pointers to GUI objects
    gui->get_widget("window", window);

    gui->get_widget("xPosSpin", xPosSpin);
    gui->get_widget("yPosSpin", yPosSpin);
    gui->get_widget("zPosSpin", zPosSpin);

    gui->get_widget("xPosCheck", xPosCheck);
    gui->get_widget("yPosCheck", yPosCheck);
    gui->get_widget("zPosCheck", zPosCheck);

    gui->get_widget("xRotSpin", xRotSpin);
    gui->get_widget("yRotSpin", yRotSpin);
    gui->get_widget("zRotSpin", zRotSpin);

    gui->get_widget("xRotCheck", xRotCheck);
    gui->get_widget("yRotCheck", yRotCheck);
    gui->get_widget("zRotCheck", zRotCheck);

    gui->get_widget("pause_play_button", pause_play_button);
    gui->get_widget("hold_release_button", hold_release_button);
    gui->get_widget("get_position_button", get_position_button);
    gui->get_widget("reset_button", reset_button);

    // Adjust the spin buttons to set min and max, and other options
    xPosSpin->set_range(-10,10);
    yPosSpin->set_range(-10,10);
    zPosSpin->set_range(-10,10);

    xPosSpin->set_increments(1,4);
    yPosSpin->set_increments(1,4);
    zPosSpin->set_increments(1,4);

    xRotSpin->set_range(-180,180);
    yRotSpin->set_range(-180,180);
    zRotSpin->set_range(-180,180);

    xRotSpin->set_increments(5,30);
    yRotSpin->set_increments(5,30);
    zRotSpin->set_increments(5,30);

    // Connect buttons to functions.
    pause_play_button->signal_toggled().connect( sigc::ptr_fun(pause_play) );
    hold_release_button->signal_toggled().connect( sigc::ptr_fun(hold_release) );
    hold_release_button->signal_clicked().connect( sigc::ptr_fun(update_constraints) );
    get_position_button->signal_clicked().connect( sigc::ptr_fun(get_position) );
    reset_button->signal_clicked().connect( sigc::ptr_fun(reset_simulation) );

  
    xPosSpin->signal_value_changed().connect( sigc::ptr_fun(update_constraints) );  
    yPosSpin->signal_value_changed().connect( sigc::ptr_fun(update_constraints) );  
    zPosSpin->signal_value_changed().connect( sigc::ptr_fun(update_constraints) );  
    xRotSpin->signal_value_changed().connect( sigc::ptr_fun(update_constraints) );
    yRotSpin->signal_value_changed().connect( sigc::ptr_fun(update_constraints) );
    zRotSpin->signal_value_changed().connect( sigc::ptr_fun(update_constraints) );

    xPosCheck->signal_toggled().connect( sigc::ptr_fun(update_constraints) );
    yPosCheck->signal_toggled().connect( sigc::ptr_fun(update_constraints) );
    zPosCheck->signal_toggled().connect( sigc::ptr_fun(update_constraints) );
    xRotCheck->signal_toggled().connect( sigc::ptr_fun(update_constraints) );
    yRotCheck->signal_toggled().connect( sigc::ptr_fun(update_constraints) );
    zRotCheck->signal_toggled().connect( sigc::ptr_fun(update_constraints) );

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

void reset_simulation()
{
    while( !reset_client.call(reset_srv) );
}

void get_position()
{
    while( !simulation_client.call(simulation_srv) );
    desired_pose = simulation_srv.response.actual_pose;
    xPosSpin->set_value(desired_pose.position.x);
    yPosSpin->set_value(desired_pose.position.y);
    zPosSpin->set_value(desired_pose.position.z + 0.025); // +0.025 is a crude hack to fix gravity error
}

void update_constraints()
{
    int x = xRotSpin->get_value();
    int y = yRotSpin->get_value();
    int z = zRotSpin->get_value();

    /*float q1 = -cos((x-z)/2) * sin(y/2);
    float q2 = -sin((x-z)/2) * sin(y/2);
    float q3 = -sin((x+z)/2) * cos(y/2);
    float q4 = sin((x+z)/2) * cos(y/2);

    float l = sqrt(q1^2 + q2^2 + q3^2 + q4^2);*/

    if( xPosCheck->get_active() )
    {
        simulation_srv.request.desired_pose.position.x = xPosSpin->get_value();
    }
    if( yPosCheck->get_active() )
    {
        simulation_srv.request.desired_pose.position.y = yPosSpin->get_value();
    }
    if( zPosCheck->get_active() )
    {
        simulation_srv.request.desired_pose.position.z = zPosSpin->get_value();
    }

/*float Q1=simulation_srv.response.actual_pose.orientation.w;
float Q2=simulation_srv.response.actual_pose.orientation.x;
float Q3=simulation_srv.response.actual_pose.orientation.y;
float Q4=simulation_srv.response.actual_pose.orientation.z;*/

//X = arctan((q1*q3 + q2*q4)

    if ( xRotCheck->get_active() )
    {
        simulation_srv.request.desired_pose.orientation.x = xRotSpin->get_value();
    }
    if( yRotCheck->get_active() )
    {
        simulation_srv.request.desired_pose.orientation.y = yRotSpin->get_value();
    }
    if( zRotCheck->get_active() )
    {
        simulation_srv.request.desired_pose.orientation.z = zRotSpin->get_value();
    }

    while( !simulation_client.call(simulation_srv) );
}
