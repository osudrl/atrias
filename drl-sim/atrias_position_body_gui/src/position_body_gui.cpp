//! @file position_body_gui.cpp
//! @brief The interface between the GUI and the robot control logic.
//! @author Devin Koepl
//! @author Colan Dray

#include <atrias_position_body_gui/position_body_gui.h>

//! @brief Initializes the GUI and controls.
//! @param argc An integer that is one more than the number of command line arguments.
//! @param argv An array of character pointers containing the command line arguments.
//! @return Returns zero upon successful completion, non-zero if an error occured.
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "atrias_gui");
    ros::NodeHandle n;
    simulation_client = n.serviceClient<drl_plugins::position_body_srv> ("/position_body_srv");
    reset_client = n.serviceClient<std_srvs::Empty> ("gazebo/reset_simulation");
    
    // Initialize service variables   
    simulation_srv.request.hold_robot = true;
    simulation_srv.request.pause_simulation = false;
    simulation_srv.request.xIsConstrained = true;
    simulation_srv.request.yIsConstrained = true;
    simulation_srv.request.zIsConstrained = true;
    simulation_srv.request.desired_pose.position.x = 0.0;
    simulation_srv.request.desired_pose.position.y = 0.0;
    simulation_srv.request.desired_pose.position.z = 1.5;

    // Initialize Gtk
    Gtk::Main gtk(argc, argv);

    // Initialize Glade
    std::string glade_gui_path = std::string(argv[0]);
    glade_gui_path = glade_gui_path.substr(0, glade_gui_path.rfind("/bin"));
    glade_gui_path = glade_gui_path.append("/src/position_body_gui.glade");

    Glib::RefPtr<Gtk::Builder> gui = Gtk::Builder::create();
    try {
        gui->add_from_file(glade_gui_path);
    } catch (const Glib::FileError& ex) {
        ROS_ERROR("File Error");
    } catch (const Gtk::BuilderError& ex) {
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

    /* TODO
    gui->get_widget("xRotSpin", xRotSpin);
    gui->get_widget("yRotSpin", yRotSpin);
    gui->get_widget("zRotSpin", zRotSpin);

    gui->get_widget("xRotCheck", xRotCheck);
    gui->get_widget("yRotCheck", yRotCheck);
    gui->get_widget("zRotCheck", zRotCheck);
    */

    gui->get_widget("pause_play_button", pause_play_button);
    gui->get_widget("hold_release_button", hold_release_button);
    gui->get_widget("get_position_button", get_position_button);
    gui->get_widget("reset_button", reset_button);

    // Set position SpinButton settings
    xPosSpin->set_range(-100, 100);
    xPosSpin->set_increments(0.01, 0);
    xPosSpin->set_value(0.0);

    yPosSpin->set_range(-100, 100);
    yPosSpin->set_increments(0.01, 0);
    yPosSpin->set_value(0.0);

    zPosSpin->set_range(-100, 100);
    zPosSpin->set_increments(0.01, 0);
    zPosSpin->set_value(1.5);

    // TODO Add rotation SpinButton settings

    // Connect buttons to functions.
    pause_play_button->signal_toggled().connect(sigc::ptr_fun(pause_play));
    hold_release_button->signal_toggled().connect(sigc::ptr_fun(hold_release));
    hold_release_button->signal_clicked().connect(sigc::ptr_fun(update_constraints));
    get_position_button->signal_clicked().connect(sigc::ptr_fun(get_position));
    reset_button->signal_clicked().connect(sigc::ptr_fun(reset_simulation));
    reset_button->signal_clicked().connect( sigc::ptr_fun(reset_simulation) );

    xPosCheck->signal_toggled().connect(sigc::ptr_fun(update_constraints));
    yPosCheck->signal_toggled().connect(sigc::ptr_fun(update_constraints));
    zPosCheck->signal_toggled().connect(sigc::ptr_fun(update_constraints));

    xPosSpin->signal_value_changed().connect(sigc::ptr_fun(update_constraints));
    yPosSpin->signal_value_changed().connect(sigc::ptr_fun(update_constraints));
    zPosSpin->signal_value_changed().connect(sigc::ptr_fun(update_constraints));

    // TODO Connect rotation CheckButtons and SpinButtons to functions.

    while (!simulation_client.call(simulation_srv));
    gtk.run(*window);

    return 0;
}

//! @brief Toggles whether the simulation is paused.
void pause_play()
{
    simulation_srv.request.pause_simulation = !simulation_srv.request.pause_simulation;

    while (!simulation_client.call(simulation_srv));
}

//! @brief Toggles whether the robot's movement is constrained. 
void hold_release()
{
    simulation_srv.request.hold_robot = !simulation_srv.request.hold_robot;

    while (!simulation_client.call(simulation_srv));
}

//! @brief Resets the simulation.
void reset_simulation()
{
    while (!reset_client.call(reset_srv));
}

//! @brief TODO
void get_position()
{
    while (!simulation_client.call(simulation_srv));
    desired_pose = simulation_srv.response.actual_pose;
    xPosSpin->set_value(desired_pose.position.x);
    yPosSpin->set_value(desired_pose.position.y);
    zPosSpin->set_value(desired_pose.position.z); //TODO +0.025?
}

//! @brief TODO
void update_constraints()
{
    simulation_srv.request.desired_pose.position.x = xPosSpin->get_value();
    simulation_srv.request.desired_pose.position.y = yPosSpin->get_value();
    simulation_srv.request.desired_pose.position.z = zPosSpin->get_value();

    /* TODO
    simulation_srv.request.desired_pose.orientation.x = xRotSpin->get_value();
    simulation_srv.request.desired_pose.orientation.y = yRotSpin->get_value();
    simulation_srv.request.desired_pose.orientation.z = zRotSpin->get_value();
    */

    simulation_srv.request.xIsConstrained = xPosCheck->get_active();
    simulation_srv.request.yIsConstrained = yPosCheck->get_active();
    simulation_srv.request.zIsConstrained = zPosCheck->get_active();

    while (!simulation_client.call(simulation_srv));
}

