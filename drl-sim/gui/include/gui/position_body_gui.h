// Devin Koepl
// Colan Dray

#include <stdlib.h>
#include <gtkmm.h>
#include <cairomm/context.h>
#include <std_srvs/Empty.h>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <drl_plugins/position_body_srv.h>

// GUI objects
Gtk::Window *window;

Gtk::SpinButton *xPosSpin;
Gtk::SpinButton *yPosSpin;
Gtk::SpinButton *zPosSpin;

Gtk::CheckButton *xPosCheck;
Gtk::CheckButton *yPosCheck;
Gtk::CheckButton *zPosCheck;

Gtk::SpinButton *xRotSpin;
Gtk::SpinButton *yRotSpin;
Gtk::SpinButton *zRotSpin;

Gtk::CheckButton *xRotCheck;
Gtk::CheckButton *yRotCheck;
Gtk::CheckButton *zRotCheck;

Gtk::ToggleButton *pause_play_button;
Gtk::ToggleButton *hold_release_button;
Gtk::Button *get_position_button;
Gtk::Button *reset_button;

ros::ServiceClient simulation_client;
drl_plugins::position_body_srv simulation_srv;
ros::ServiceClient reset_client;
std_srvs::Empty reset_srv;

geometry_msgs::Pose desired_pose;

void pause_play();
void hold_release();
void get_position();
void reset_simulation();
void update_constraints();
